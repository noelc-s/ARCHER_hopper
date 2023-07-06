#include"Types.h"

using namespace Hopper_t;

class Graph {
  public:

    struct graph {
      vertex_array_t V;  // std::vector of vertices
      edge_array_t E;    // std::vector of edges
    } G;

    Graph(vertex_array_t v_list, edge_array_t e_list) {
      G.V = v_list;
      G.E = e_list;
    }
    ~Graph() {}

    // membership oracle
    bool isInPolytope(vector_t x, Polytope P);

};

