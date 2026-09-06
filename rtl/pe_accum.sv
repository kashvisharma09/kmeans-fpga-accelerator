// =====================================================================
// pe_accum.sv
// Each PE owns a private set of K accumulators (sum_x, sum_y, count).
// Because every lane only ever writes to its OWN accumulator bank,
// there's never a read-modify-write hazard between lanes running in
// parallel -- no arbitration or atomics needed. kmeans_top later
// reduces (sums) the NUM_PE private banks together to get the true
// per-cluster totals ("map" locally, "reduce" globally).
//
// sum_x/sum_y/count are PACKED [K-1:0] arrays so each is a single,
// portable vector -- kmeans_top can connect one whole bank per PE
// with a plain element-of-array connection, no array slicing needed.
// =====================================================================
module pe_accum
    import kmeans_pkg::*;
(
    input  logic            clk,
    input  logic            rst_n,
    input  logic            clr,             // synchronous clear (start of each iteration)
    input  logic            valid,
    input  clid_t           cluster_id,
    input  point_t          point,
    output sum_t   [K-1:0]  sum_x,
    output sum_t   [K-1:0]  sum_y,
    output count_t [K-1:0]  count
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clr) begin
            sum_x <= '0;
            sum_y <= '0;
            count <= '0;
        end else if (valid) begin
            sum_x[cluster_id] <= sum_x[cluster_id] + sum_t'(point.x);
            sum_y[cluster_id] <= sum_y[cluster_id] + sum_t'(point.y);
            count[cluster_id] <= count[cluster_id] + 1'b1;
        end
    end

endmodule : pe_accum
