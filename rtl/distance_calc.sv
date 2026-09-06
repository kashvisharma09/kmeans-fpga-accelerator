// =====================================================================
// distance_calc.sv
// Computes the squared Euclidean distance between a data point and a
// centroid:  dist_sq = (px-cx)^2 + (py-cy)^2
// (we compare squared distances, so the sqrt is never needed for
// nearest-centroid assignment -- this saves a lot of hardware.)
// One pipeline register -> 1 cycle of latency, valid one cycle after en.
// =====================================================================
module distance_calc
    import kmeans_pkg::*;
(
    input  logic   clk,
    input  logic   rst_n,
    input  logic   en,
    input  point_t point,
    input  point_t centroid,
    output dist_t  dist_sq,
    output logic   valid
);

    // signed, one extra bit so subtraction of two unsigned coords can't wrap
    logic signed [DATA_WIDTH:0]     dx, dy;
    logic        [2*DATA_WIDTH+1:0] dx2, dy2;
    logic        [2*DATA_WIDTH+2:0] sum_sq;

    always_comb begin
        dx     = $signed({1'b0, point.x})    - $signed({1'b0, centroid.x});
        dy     = $signed({1'b0, point.y})    - $signed({1'b0, centroid.y});
        dx2    = dx * dx;
        dy2    = dy * dy;
        sum_sq = dx2 + dy2;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dist_sq <= '0;
            valid   <= 1'b0;
        end else begin
            valid <= en;
            if (en) dist_sq <= sum_sq; // implicit zero-extend into wider dist_t
        end
    end

endmodule : distance_calc
