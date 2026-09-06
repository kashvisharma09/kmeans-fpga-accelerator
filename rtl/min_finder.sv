// =====================================================================
// min_finder.sv
// Combinational argmin over K squared-distance values: reports which
// centroid a point is closest to. K is small (a handful of clusters),
// so a simple linear-scan comparator chain is the right amount of
// hardware; for large K this would become a comparator tree instead.
//
// dist_arr is a PACKED array (K-1:0] of dist_t words, not an unpacked
// array -- packed arrays behave like plain vectors, which keeps this
// module's port trivially portable across simulators/synthesis tools.
// =====================================================================
module min_finder
    import kmeans_pkg::*;
(
    input  dist_t [K-1:0] dist_arr,
    output clid_t         min_id,
    output dist_t         min_dist
);

    always_comb begin
        min_id   = '0;
        min_dist = dist_arr[0];
        for (int i = 1; i < K; i++) begin
            if (dist_arr[i] < min_dist) begin
                min_dist = dist_arr[i];
                min_id   = clid_t'(i);
            end
        end
    end

endmodule : min_finder
