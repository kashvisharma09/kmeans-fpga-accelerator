// =====================================================================
// kmeans_tb.sv
// Loads a synthetic 2D dataset (four loose blobs) into the accelerator,
// runs it to convergence, and cross-checks the hardware's final
// centroids/counts against a software model of the exact same
// algorithm (same integer arithmetic, same initial centroids) run in
// the testbench. Also dumps a VCD for waveform viewing.
// =====================================================================
`timescale 1ns/1ps

module kmeans_tb;
    import kmeans_pkg::*;

    logic clk = 0;
    logic rst_n = 0;
    always #5 clk = ~clk; // 100 MHz

    logic   start;
    logic   load_pt_en;
    ptid_t  load_pt_addr;
    point_t load_pt_data;
    logic   load_ct_en;
    clid_t  load_ct_addr;
    point_t load_ct_data;

    logic   busy, done;
    logic [$clog2(MAX_ITER+1)-1:0] iter_count;
    point_t [K-1:0] centroids_out;
    count_t [K-1:0] counts_out;

    kmeans_top dut (
        .clk (clk), .rst_n(rst_n),
        .start(start),
        .load_pt_en(load_pt_en), .load_pt_addr(load_pt_addr), .load_pt_data(load_pt_data),
        .load_ct_en(load_ct_en), .load_ct_addr(load_ct_addr), .load_ct_data(load_ct_data),
        .busy(busy), .done(done), .iter_count(iter_count),
        .centroids_out(centroids_out), .counts_out(counts_out)
    );

    // ------------------------------------------------------------
    // Synthetic dataset: 4 loose blobs of 8 points each (32 total),
    // roughly centered at (10,10) (10,90) (90,10) (90,90).
    // ------------------------------------------------------------
    point_t dataset [NUM_POINTS];
    point_t init_centroids [K];

    function automatic point_t mkpt(coord_t xx, coord_t yy);
        mkpt.x = xx;
        mkpt.y = yy;
    endfunction

    initial begin
        // Blob 0 ~ (10,10)
        dataset[0] = mkpt(12'd8, 12'd9);
        dataset[1] = mkpt(12'd11, 12'd14);
        dataset[2] = mkpt(12'd9, 12'd7);
        dataset[3] = mkpt(12'd13, 12'd12);
        dataset[4] = mkpt(12'd7, 12'd15);
        dataset[5] = mkpt(12'd10, 12'd11);
        dataset[6] = mkpt(12'd6, 12'd8);
        dataset[7] = mkpt(12'd14, 12'd10);
        // Blob 1 ~ (10,90)
        dataset[8] = mkpt(12'd9, 12'd88);
        dataset[9] = mkpt(12'd12, 12'd92);
        dataset[10] = mkpt(12'd7, 12'd85);
        dataset[11] = mkpt(12'd11, 12'd95);
        dataset[12] = mkpt(12'd15, 12'd90);
        dataset[13] = mkpt(12'd8, 12'd93);
        dataset[14] = mkpt(12'd13, 12'd87);
        dataset[15] = mkpt(12'd10, 12'd91);
        // Blob 2 ~ (90,10)
        dataset[16] = mkpt(12'd88, 12'd8);
        dataset[17] = mkpt(12'd92, 12'd12);
        dataset[18] = mkpt(12'd85, 12'd9);
        dataset[19] = mkpt(12'd95, 12'd13);
        dataset[20] = mkpt(12'd90, 12'd7);
        dataset[21] = mkpt(12'd87, 12'd14);
        dataset[22] = mkpt(12'd93, 12'd10);
        dataset[23] = mkpt(12'd89, 12'd11);
        // Blob 3 ~ (90,90)
        dataset[24] = mkpt(12'd88, 12'd90);
        dataset[25] = mkpt(12'd91, 12'd87);
        dataset[26] = mkpt(12'd94, 12'd93);
        dataset[27] = mkpt(12'd86, 12'd94);
        dataset[28] = mkpt(12'd92, 12'd89);
        dataset[29] = mkpt(12'd89, 12'd92);
        dataset[30] = mkpt(12'd95, 12'd88);
        dataset[31] = mkpt(12'd90, 12'd95);

        // Deliberately-imperfect initial guesses (not the true blob centers)
        init_centroids[0] = mkpt(12'd0, 12'd0);
        init_centroids[1] = mkpt(12'd0, 12'd50);
        init_centroids[2] = mkpt(12'd50, 12'd0);
        init_centroids[3] = mkpt(12'd50, 12'd50);
    end

    // ------------------------------------------------------------
    // Software reference model: identical algorithm, identical
    // integer arithmetic, run on the same dataset/initial centroids.
    // ------------------------------------------------------------
    point_t ref_centroids [K];
    int     ref_iters;

    task automatic run_reference();
        point_t cur [K];
        point_t nxt [K];
        int     sx [K], sy [K], cnt [K];
        int     dx, dy, d, best_d, best_k;
        bit     conv;
        bit     stop;
        int     it;
        point_t pt, ctr;
        int     px, py;

        for (int k = 0; k < K; k++) cur[k] = init_centroids[k];

        stop = 1'b0;
        for (it = 0; it < MAX_ITER && !stop; it++) begin
            for (int k = 0; k < K; k++) begin sx[k]=0; sy[k]=0; cnt[k]=0; end

            for (int i = 0; i < NUM_POINTS; i++) begin
                pt     = dataset[i];
                px     = int'(pt.x);
                py     = int'(pt.y);
                best_d = 32'h7fffffff;
                best_k = 0;
                for (int k = 0; k < K; k++) begin
                    ctr = cur[k];
                    dx  = px - int'(ctr.x);
                    dy  = py - int'(ctr.y);
                    d   = dx*dx + dy*dy;
                    if (d < best_d) begin best_d = d; best_k = k; end
                end
                sx[best_k] += px;
                sy[best_k] += py;
                cnt[best_k]++;
            end

            conv = 1'b1;
            for (int k = 0; k < K; k++) begin
                if (cnt[k] != 0) begin
                    ctr.x = coord_t'(sx[k] / cnt[k]);
                    ctr.y = coord_t'(sy[k] / cnt[k]);
                    nxt[k] = ctr;
                end else begin
                    nxt[k] = cur[k];
                end
                if (nxt[k] != cur[k]) conv = 1'b0;
                cur[k] = nxt[k];
            end
            if (conv) stop = 1'b1;
        end
        for (int k = 0; k < K; k++) ref_centroids[k] = cur[k]; // element-wise (not whole-array) copy
        ref_iters = it;
    endtask

    // ------------------------------------------------------------
    // Drive the DUT
    // ------------------------------------------------------------
    initial begin
        $dumpfile("kmeans.vcd");
        $dumpvars(0, kmeans_tb);

        start        = 0;
        load_pt_en   = 0;
        load_ct_en   = 0;
        load_pt_addr = '0;
        load_ct_addr = '0;
        load_pt_data = '0;
        load_ct_data = '0;

        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // Load dataset
        for (int i = 0; i < NUM_POINTS; i++) begin
            @(posedge clk);
            load_pt_en   <= 1;
            load_pt_addr <= ptid_t'(i);
            load_pt_data <= dataset[i];
        end
        @(posedge clk);
        load_pt_en <= 0;

        // Load initial centroids
        for (int k = 0; k < K; k++) begin
            @(posedge clk);
            load_ct_en   <= 1;
            load_ct_addr <= clid_t'(k);
            load_ct_data <= init_centroids[k];
        end
        @(posedge clk);
        load_ct_en <= 0;

        // Kick off clustering
        @(posedge clk);
        start <= 1;
        @(posedge clk);
        start <= 0;

        wait (done);
        @(posedge clk);

        // ---------------- Report + self-check ----------------
        begin
            bit     pass;
            point_t hw_c, sw_c;
            run_reference();

            $display("\n================ K-MEANS ACCELERATOR RESULT ================");
            $display("Hardware converged after %0d iteration(s)", iter_count + 1);
            for (int k = 0; k < K; k++) begin
                hw_c = centroids_out[k];
                $display("  cluster %0d: centroid=(%0d,%0d)  points=%0d",
                          k, hw_c.x, hw_c.y, counts_out[k]);
            end

            $display("\n---------------- Software reference model ----------------");
            for (int k = 0; k < K; k++) begin
                sw_c = ref_centroids[k];
                $display("  cluster %0d: centroid=(%0d,%0d)", k, sw_c.x, sw_c.y);
            end

            pass = 1;
            for (int k = 0; k < K; k++) begin
                hw_c = centroids_out[k];
                if (hw_c != ref_centroids[k]) pass = 0;
            end

            if (pass) $display("\n*** TEST PASSED: hardware matches software reference bit-exact ***\n");
            else      $display("\n*** TEST FAILED: hardware/software mismatch ***\n");
        end

        #20 $finish;
    end

    // Safety timeout
    initial begin
        #100000;
        $display("ERROR: simulation timeout - DUT never asserted done");
        $finish;
    end

endmodule : kmeans_tb
