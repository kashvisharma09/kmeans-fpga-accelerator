// =====================================================================
// kmeans_top.sv
// GPU-style K-means (Lloyd's algorithm) accelerator.
//
// Pipeline per iteration:
//   S_ASSIGN : NUM_PE lanes stream points from memory in parallel,
//              round-robin (lane p gets points p, p+NUM_PE, p+2*NUM_PE, ...).
//              Each lane computes nearest centroid + accumulates into
//              its own private per-cluster sum_x/sum_y/count bank.
//   S_REDUCE : the NUM_PE private banks are summed ("reduced") into one
//              global sum_x/sum_y/count per cluster.
//   S_UPDATE : new centroid = sum / count (mean of assigned points).
//   S_CHECK  : if centroids stopped moving (or MAX_ITER hit) -> S_DONE,
//              else clear accumulators and go back to S_ASSIGN.
//
// All K-wide and NUM_PE-wide per-cluster signals use PACKED [K-1:0]
// (or [NUM_PE-1:0]) arrays rather than unpacked arrays. This keeps
// every port connection a plain vector -- no array slicing, no
// whole-unpacked-array continuous assignment -- which is by far the
// most portable style across simulators and, importantly, across
// Vivado's synthesizer as well as its simulator.
// =====================================================================
module kmeans_top
    import kmeans_pkg::*;
(
    input  logic   clk,
    input  logic   rst_n,

    input  logic   start,              // pulse to (re)start clustering

    // --- load interface: write dataset / initial centroids while idle ---
    input  logic   load_pt_en,
    input  ptid_t  load_pt_addr,
    input  point_t load_pt_data,

    input  logic   load_ct_en,
    input  clid_t  load_ct_addr,
    input  point_t load_ct_data,

    output logic   busy,
    output logic   done,
    output logic [$clog2(MAX_ITER+1)-1:0] iter_count,

    output point_t [K-1:0] centroids_out,
    output count_t [K-1:0] counts_out
);

    // ---------------- Point memory (dynamic-index read/write, standard RAM style) ----------------
    point_t point_mem [NUM_POINTS];

    always_ff @(posedge clk) begin
        if (load_pt_en) point_mem[load_pt_addr] <= load_pt_data;
    end

    // ---------------- Centroid registers ----------------
    point_t [K-1:0] centroids;

    // ---------------- FSM ----------------
    typedef enum logic [2:0] {S_IDLE, S_ASSIGN, S_REDUCE, S_UPDATE, S_CHECK, S_DONE} state_e;
    state_e state;

    localparam int RIDX_W = (ROUNDS >= 1) ? $clog2(ROUNDS+1) : 1;
    logic [RIDX_W-1:0] round_idx, safe_ridx;
    logic pe_en, clr_accum;

    assign pe_en     = (state == S_ASSIGN) && (round_idx < ROUNDS);
    assign safe_ridx = (round_idx < ROUNDS) ? round_idx : ROUNDS-1; // clamp for addr calc only

    // ---------------- PE array + private per-lane accumulators ----------------
    point_t pe_point   [NUM_PE];
    point_t pe_point_o [NUM_PE];
    clid_t  pe_cluster [NUM_PE];
    dist_t  pe_mindist [NUM_PE];
    logic   pe_valid   [NUM_PE];

    sum_t   [K-1:0] loc_sum_x [NUM_PE];
    sum_t   [K-1:0] loc_sum_y [NUM_PE];
    count_t [K-1:0] loc_count [NUM_PE];

    genvar p;
    generate
        for (p = 0; p < NUM_PE; p++) begin : gen_pe
            logic [PTID_WIDTH-1:0] addr;
            assign addr        = ptid_t'(safe_ridx) * NUM_PE + p;
            assign pe_point[p] = point_mem[addr];

            pe u_pe (
                .clk              (clk),
                .rst_n            (rst_n),
                .en               (pe_en),
                .point            (pe_point[p]),
                .centroids        (centroids),
                .point_out        (pe_point_o[p]),
                .assigned_cluster (pe_cluster[p]),
                .min_dist         (pe_mindist[p]),
                .valid            (pe_valid[p])
            );

            pe_accum u_acc (
                .clk        (clk),
                .rst_n      (rst_n),
                .clr        (clr_accum),
                .valid      (pe_valid[p]),
                .cluster_id (pe_cluster[p]),
                .point      (pe_point_o[p]),
                .sum_x      (loc_sum_x[p]),
                .sum_y      (loc_sum_y[p]),
                .count      (loc_count[p])
            );
        end
    endgenerate

    // ---------------- Reduce: sum the NUM_PE private banks ----------------
    sum_t   [K-1:0] red_sum_x;
    sum_t   [K-1:0] red_sum_y;
    count_t [K-1:0] red_count;

    always_comb begin
        red_sum_x = '0;
        red_sum_y = '0;
        red_count = '0;
        for (int k = 0; k < K; k++) begin
            for (int lane = 0; lane < NUM_PE; lane++) begin
                red_sum_x[k] += loc_sum_x[lane][k];
                red_sum_y[k] += loc_sum_y[lane][k];
                red_count[k] += loc_count[lane][k];
            end
        end
    end

    sum_t   [K-1:0] g_sum_x;
    sum_t   [K-1:0] g_sum_y;
    count_t [K-1:0] g_count;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_sum_x <= '0;
            g_sum_y <= '0;
            g_count <= '0;
        end else if (state == S_REDUCE) begin
            g_sum_x <= red_sum_x;
            g_sum_y <= red_sum_y;
            g_count <= red_count;
        end
    end

    // ---------------- Update: new centroid = mean of assigned points ----------------
    point_t [K-1:0] new_centroids;
    logic           converged;

    always_comb begin
        point_t nc; // local unpacked scratch value - avoids writing a struct
                    // *field* through a variably-indexed packed array, which
                    // some tools (incl. some simulators) don't support well
        converged = 1'b1;
        for (int k = 0; k < K; k++) begin
            if (g_count[k] != 0) begin
                nc.x = coord_t'(g_sum_x[k] / sum_t'(g_count[k]));
                nc.y = coord_t'(g_sum_y[k] / sum_t'(g_count[k]));
            end else begin
                nc = centroids[k]; // empty cluster: leave centroid where it was
            end
            new_centroids[k] = nc;
            if (nc != centroids[k]) converged = 1'b0;
        end
    end

    // `converged` (above) is combinational: new_centroids vs the CURRENT
    // (not-yet-updated) centroids register. It is only meaningful during
    // the S_UPDATE cycle, the one cycle where `centroids` still holds the
    // previous iteration's values. We latch it into converged_r in that
    // same cycle so S_CHECK compares against a stable, correctly-timed
    // result instead of re-evaluating `converged` a cycle later (by which
    // point centroids would already equal new_centroids trivially).
    logic converged_r;

    // ---------------- FSM sequencing ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            round_idx   <= '0;
            iter_count  <= '0;
            clr_accum   <= 1'b0;
            centroids   <= '0;
            converged_r <= 1'b0;
        end else begin
            clr_accum <= 1'b0;
            unique case (state)
                S_IDLE: begin
                    round_idx  <= '0;
                    iter_count <= '0;
                    if (load_ct_en) centroids[load_ct_addr] <= load_ct_data;
                    if (start) begin
                        state     <= S_ASSIGN;
                        clr_accum <= 1'b1;
                    end
                end

                S_ASSIGN: begin
                    round_idx <= round_idx + 1'b1;
                    if (round_idx == ROUNDS) state <= S_REDUCE; // drain cycle finished
                end

                S_REDUCE: state <= S_UPDATE;

                S_UPDATE: begin
                    converged_r <= converged;    // sample BEFORE centroids changes
                    centroids   <= new_centroids;
                    state       <= S_CHECK;
                end

                S_CHECK: begin
                    if (converged_r || (iter_count == MAX_ITER-1)) begin
                        state <= S_DONE;
                    end else begin
                        iter_count <= iter_count + 1'b1;
                        round_idx  <= '0;
                        clr_accum  <= 1'b1;
                        state      <= S_ASSIGN;
                    end
                end

                S_DONE: begin
                    if (start) begin
                        state      <= S_ASSIGN;
                        round_idx  <= '0;
                        iter_count <= '0;
                        clr_accum  <= 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    assign busy         = (state != S_IDLE) && (state != S_DONE);
    assign done          = (state == S_DONE);
    assign centroids_out = centroids;
    assign counts_out    = g_count;

endmodule : kmeans_top
