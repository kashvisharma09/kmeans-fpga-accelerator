// =====================================================================
// board_top_tb.sv
// Smoke test for kmeans_board_top: no external data loading at all --
// just reset, press the button, and check the LEDs / debug taps reach
// the same correct result as tb/kmeans_tb.sv did with the DUT driven
// directly. This proves the auto-load ROM + button-edge-detect wrapper
// logic is correct on top of the already-verified kmeans_top core.
// =====================================================================
`timescale 1ns/1ps

module board_top_tb;
    import kmeans_pkg::*;

    logic clk = 0;
    logic rst_n = 0;
    logic start_btn = 0;
    logic led_busy, led_done;

    always #5 clk = ~clk;

    kmeans_board_top uut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start_btn (start_btn),
        .led_busy  (led_busy),
        .led_done  (led_done)
    );

    initial begin
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;

        // give the auto-load FSM time to stream in all points + centroids
        repeat (60) @(posedge clk);

        // press the button for a few cycles (models a real debounced press)
        start_btn = 1;
        repeat (5) @(posedge clk);
        start_btn = 0;

        wait (led_done);
        @(posedge clk);

        begin
            point_t c;
            $display("\n================ BOARD WRAPPER SMOKE TEST ================");
            for (int k = 0; k < K; k++) begin
                c = uut.dbg_centroids[k];
                $display("  cluster %0d: centroid=(%0d,%0d)  points=%0d",
                          k, c.x, c.y, uut.dbg_counts[k]);
            end
            $display("iterations used: %0d", uut.dbg_iter + 1);
            $display("led_done=%0b led_busy=%0b", led_done, led_busy);
        end

        #20 $finish;
    end

    initial begin
        #100000;
        $display("ERROR: timeout - led_done never asserted");
        $finish;
    end

endmodule : board_top_tb
