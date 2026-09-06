// =====================================================================
// kmeans_pkg.sv
// Global parameters and types for the K-means GPU-style accelerator.
// Change these to resize the problem (dataset size, #clusters,
// #parallel lanes, coordinate precision) without touching any other
// file.
// =====================================================================
package kmeans_pkg;

    parameter int DATA_WIDTH  = 12;   // bits per coordinate (unsigned, 0..4095)
    parameter int DIM         = 2;    // dimensionality (this project uses x,y)
    parameter int K           = 4;    // number of clusters
    parameter int NUM_POINTS  = 32;   // dataset size (must be multiple of NUM_PE)
    parameter int NUM_PE      = 4;    // parallel "lanes" / processing elements
    parameter int MAX_ITER    = 16;   // safety cap on Lloyd's-algorithm iterations

    // Derived widths (sized generously so nothing overflows for the
    // parameters above; recompute if you change DATA_WIDTH/NUM_POINTS/K).
    parameter int DIST_WIDTH  = 2*DATA_WIDTH + 4;                 // squared-distance sum
    parameter int SUM_WIDTH   = DATA_WIDTH + $clog2(NUM_POINTS) + 2; // coordinate accumulator
    parameter int COUNT_WIDTH = $clog2(NUM_POINTS) + 2;
    parameter int PTID_WIDTH  = (NUM_POINTS > 1) ? $clog2(NUM_POINTS) : 1;
    parameter int CLID_WIDTH  = (K > 1) ? $clog2(K) : 1;
    parameter int ROUNDS      = NUM_POINTS / NUM_PE; // dispatch rounds in ASSIGN phase

    typedef logic [DATA_WIDTH-1:0]  coord_t;
    typedef logic [DIST_WIDTH-1:0]  dist_t;
    typedef logic [CLID_WIDTH-1:0]  clid_t;
    typedef logic [PTID_WIDTH-1:0]  ptid_t;
    typedef logic [SUM_WIDTH-1:0]   sum_t;
    typedef logic [COUNT_WIDTH-1:0] count_t;

    typedef struct packed {
        coord_t x;
        coord_t y;
    } point_t;

endpackage : kmeans_pkg
