#include "Types.h"
#include <Polyhedron.h>

using namespace Hopper_t;

class Graph {
  public:

    vertex_array_t V;  // std::vector of vertices
    edge_array_t E;    // std::vector of edges

    Graph(vertex_array_t v_list, edge_array_t e_list) : V(v_list), E(e_list) {}
    ~Graph() {}

    // size of V
    int getVsize();
    // size of E
    int getEsize();

    // membership oracle
    bool isInPolytope(vector_t x, Polytope_H P);

    // convert polytope V rep to H rep
    Polytope_H Vrep_to_Hrep(Polytope_V P_vrep);
    
    // convert polytope H rep to V rep
    Polytope_V Hrep_to_Vrep(Polytope_H P_hrep);

};

