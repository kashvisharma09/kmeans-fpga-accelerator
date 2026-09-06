// =====================================================================
// kmeans_board_top.sv
// Standalone, board-ready wrapper around kmeans_top.
//
// This is what you'd actually put in an XDC-constrained project and
// program onto an FPGA:
//   - No host PC / testbench needed: the same 32-point synthetic
//     dataset from the testbench is hardcoded into an internal ROM
//     and auto-loaded into kmeans_top right after reset.
//   - A single debounced button pulse (start_btn) kicks off clustering.
//   - busy/done are broken out to LEDs.
//   - Key internal results (state, iter_count, centroids, counts) are
//     tagged with (* mark_debug = "true" *) so Vivado's "Set up Debug"
//     wizard (or manually inserting an ILA IP) can wire them straight
//     into an Integrated Logic Analyzer -- this lets you watch the
//     FSM and the final centroids live on real hardware, the same way
//     you'd watch waveforms in simulation, without needing a UART or
//     any other I/O.
//
// Swap the pin names in the matching .xdc for whatever board you end
// up using -- this module itself is board-independent.
// =====================================================================
module kmeans_board_top
    import kmeans_pkg::*;
(
    input  logic clk,        // board oscillator (constrain + buffer via XDC)
    input  logic rst_n,      // active-low reset button
    input  logic start_btn,  // momentary push button: start clustering
    output logic led_busy,
    output logic led_done
);

    // -----------------------------------------------------------------
    // Hardcoded dataset ROM: identical 4-blob, 32-point dataset used in
    // tb/kmeans_tb.sv, plus the same deliberately-off initial centroid
    // guesses. Implemented as simple combinational lookup functions --
    // Vivado will map these to distributed ROM / LUTs automatically.
    // -----------------------------------------------------------------
    function automatic point_t rom_pt(input int idx);
        point_t p;
        unique case (idx)
            0:  begin p.x = coord_t'(8);  p.y = coord_t'(9);  end
            1:  begin p.x = coord_t'(11); p.y = coord_t'(14); end
            2:  begin p.x = coord_t'(9);  p.y = coord_t'(7);  end
            3:  begin p.x = coord_t'(13); p.y = coord_t'(12); end
            4:  begin p.x = coord_t'(7);  p.y = coord_t'(15); end
            5:  begin p.x = coord_t'(10); p.y = coord_t'(11); end
            6:  begin p.x = coord_t'(6);  p.y = coord_t'(8);  end
            7:  begin p.x = coord_t'(14); p.y = coord_t'(10); end
            8:  begin p.x = coord_t'(9);  p.y = coord_t'(88); end
            9:  begin p.x = coord_t'(12); p.y = coord_t'(92); end
            10: begin p.x = coord_t'(7);  p.y = coord_t'(85); end
            11: begin p.x = coord_t'(11); p.y = coord_t'(95); end
            12: begin p.x = coord_t'(15); p.y = coord_t'(90); end
            13: begin p.x = coord_t'(8);  p.y = coord_t'(93); end
            14: begin p.x = coord_t'(13); p.y = coord_t'(87); end
            15: begin p.x = coord_t'(10); p.y = coord_t'(91); end
            16: begin p.x = coord_t'(88); p.y = coord_t'(8);  end
            17: begin p.x = coord_t'(92); p.y = coord_t'(12); end
            18: begin p.x = coord_t'(85); p.y = coord_t'(9);  end
            19: begin p.x = coord_t'(95); p.y = coord_t'(13); end
            20: begin p.x = coord_t'(90); p.y = coord_t'(7);  end
            21: begin p.x = coord_t'(87); p.y = coord_t'(14); end
            22: begin p.x = coord_t'(93); p.y = coord_t'(10); end
            23: begin p.x = coord_t'(89); p.y = coord_t'(11); end
            24: begin p.x = coord_t'(88); p.y = coord_t'(90); end
            25: begin p.x = coord_t'(91); p.y = coord_t'(87); end
            26: begin p.x = coord_t'(94); p.y = coord_t'(93); end
            27: begin p.x = coord_t'(86); p.y = coord_t'(94); end
            28: begin p.x = coord_t'(92); p.y = coord_t'(89); end
            29: begin p.x = coord_t'(89); p.y = coord_t'(92); end
            30: begin p.x = coord_t'(95); p.y = coord_t'(88); end
            31: begin p.x = coord_t'(90); p.y = coord_t'(95); end
            default: begin p.x = coord_t'(0); p.y = coord_t'(0); end
        endcase
        rom_pt = p;
    endfunction

    function automatic point_t rom_ctr(input int idx);
        point_t p;
        unique case (idx)
            0: begin p.x = coord_t'(0);  p.y = coord_t'(0);  end
            1: begin p.x = coord_t'(0);  p.y = coord_t'(50); end
            2: begin p.x = coord_t'(50); p.y = coord_t'(0);  end
            3: begin p.x = coord_t'(50); p.y = coord_t'(50); end
            default: begin p.x = coord_t'(0); p.y = coord_t'(0); end
        endcase
        rom_ctr = p;
    endfunction

    // -----------------------------------------------------------------
    // Auto-load FSM: on reset, stream the ROM into kmeans_top's load
    // interface, then wait for a debounced start-button pulse.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {L_PTS, L_CTR, L_RUN} lstate_e;
    lstate_e lstate;

    ptid_t li;
    clid_t lk;

    logic   load_pt_en;
    ptid_t  load_pt_addr;
    point_t load_pt_data;
    logic   load_ct_en;
    clid_t  load_ct_addr;
    point_t load_ct_data;
    logic   start;

    // simple 2-flop synchronizer + edge detect for the async push button
    logic btn_ff1, btn_ff2, btn_ff3;
    logic start_pulse;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_ff1 <= 1'b0; btn_ff2 <= 1'b0; btn_ff3 <= 1'b0;
        end else begin
            btn_ff1 <= start_btn;
            btn_ff2 <= btn_ff1;
            btn_ff3 <= btn_ff2;
        end
    end
    assign start_pulse = btn_ff2 & ~btn_ff3;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lstate       <= L_PTS;
            li           <= '0;
            lk           <= '0;
            load_pt_en   <= 1'b0;
            load_ct_en   <= 1'b0;
            start        <= 1'b0;
        end else begin
            load_pt_en <= 1'b0;
            load_ct_en <= 1'b0;
            start      <= 1'b0;
            unique case (lstate)
                L_PTS: begin
                    load_pt_en   <= 1'b1;
                    load_pt_addr <= li;
                    load_pt_data <= rom_pt(int'(li));
                    if (li == ptid_t'(NUM_POINTS-1)) begin
                        lk     <= '0;
                        lstate <= L_CTR;
                    end else begin
                        li <= li + 1'b1;
                    end
                end
                L_CTR: begin
                    load_ct_en   <= 1'b1;
                    load_ct_addr <= lk;
                    load_ct_data <= rom_ctr(int'(lk));
                    if (lk == clid_t'(K-1)) begin
                        lstate <= L_RUN;
                    end else begin
                        lk <= lk + 1'b1;
                    end
                end
                L_RUN: begin
                    if (start_pulse) start <= 1'b1;
                end
                default: lstate <= L_PTS;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // The accelerator itself
    // -----------------------------------------------------------------
    logic   busy, done;
    logic [$clog2(MAX_ITER+1)-1:0] iter_count;
    point_t [K-1:0] centroids_out;
    count_t [K-1:0] counts_out;

    kmeans_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .load_pt_en   (load_pt_en),
        .load_pt_addr (load_pt_addr),
        .load_pt_data (load_pt_data),
        .load_ct_en   (load_ct_en),
        .load_ct_addr (load_ct_addr),
        .load_ct_data (load_ct_data),
        .busy         (busy),
        .done         (done),
        .iter_count   (iter_count),
        .centroids_out(centroids_out),
        .counts_out   (counts_out)
    );

    // -----------------------------------------------------------------
    // ILA debug taps. After synthesis, Tools -> Set up Debug (or mark
    // these nets in the schematic viewer) will auto-insert an ILA core
    // wired to `clk` and capturing every signal below.
    // -----------------------------------------------------------------
    (* mark_debug = "true" *) logic                            dbg_busy;
    (* mark_debug = "true" *) logic                            dbg_done;
    (* mark_debug = "true" *) logic [$clog2(MAX_ITER+1)-1:0]   dbg_iter;
    (* mark_debug = "true" *) point_t [K-1:0]                  dbg_centroids;
    (* mark_debug = "true" *) count_t [K-1:0]                  dbg_counts;
    (* mark_debug = "true" *) lstate_e                         dbg_lstate;

    assign dbg_busy      = busy;
    assign dbg_done      = done;
    assign dbg_iter      = iter_count;
    assign dbg_centroids = centroids_out;
    assign dbg_counts    = counts_out;
    assign dbg_lstate    = lstate;

    assign led_busy = busy;
    assign led_done = done;

endmodule : kmeans_board_top
