// =====================================================================
// pe.sv  (Processing Element)
// This is the "core" in our small GPU-style array. Each PE receives a
// data point and the full set of K centroids, computes the distance to
// every centroid IN PARALLEL (one distance_calc per cluster), and picks
// the nearest one. NUM_PE of these run side by side in kmeans_top,
// each streaming through its own share of the dataset -- the same
// SIMT-ish "many lanes, same program, different data" idea a real GPU
// uses, just applied to K-means instead of pixels.
//
// Latency: 1 cycle (matches distance_calc's single pipeline register).
// centroids is a PACKED [K-1:0] array so it can be broadcast to every
// PE with a single, portable vector connection.
// =====================================================================
module pe
    import kmeans_pkg::*;
(
    input  logic          clk,
    input  logic          rst_n,
    input  logic          en,                 // new point presented this cycle
    input  point_t        point,
    input  point_t [K-1:0] centroids,
    output point_t        point_out,          // point, delayed to line up with valid
    output clid_t         assigned_cluster,
    output dist_t         min_dist,
    output logic          valid
);

    dist_t [K-1:0] dist_arr;
    logic          dc_valid [K];

    genvar g;
    generate
        for (g = 0; g < K; g++) begin : gen_dist
            distance_calc u_dc (
                .clk      (clk),
                .rst_n    (rst_n),
                .en       (en),
                .point    (point),
                .centroid (centroids[g]),
                .dist_sq  (dist_arr[g]),
                .valid    (dc_valid[g])
            );
        end
    endgenerate

    min_finder u_min (
        .dist_arr (dist_arr),
        .min_id   (assigned_cluster),
        .min_dist (min_dist)
    );

    assign valid = dc_valid[0]; // all K lanes share identical en/latency

    // delay the point by the same 1 cycle so it lines up with `valid`
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)   point_out <= '0;
        else if (en)  point_out <= point;
    end

endmodule : pe
