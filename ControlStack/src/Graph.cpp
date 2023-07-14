#include "../inc/Graph.h"
#include<iostream>

///////////////////////////////////////////////////////////////////////

//  return the size of V
int Graph::getVsize() {
  return V.size();
}

// return the size of E
int Graph::getEsize() {
  return E.size();
}

///////////////////////////////////////////////////////////////////////


// membership oracle
bool Graph::isInPolytope(vector_t x, Polytope_H P) {
  
  bool is_member=true;

  matrix_t C_;
  vector_t d_;
  C_ = P.C;
  d_ = P.d;

  int r = C_.rows();
  int c = C_.cols();
  row_vector_t row(c);
  scalar_t val;

  for (int i=0; i<r; i++) {
    row << C_.block(i,0,1,c);
    val = d_(i);
    
    if (row.dot(x) > val) {
      is_member=false;
      break;
    }
  }
  return is_member;
}

//////////////////////////////////////////////////////////

// convert polytope V rep to H rep
Polytope_H Graph::Vrep_to_Hrep(Polytope_V P_vrep) {

  Eigen::Polyhedron poly;
  
  int num_verts = P_vrep.V.size();
  int dim_verts = P_vrep.V[0].size();

  // since we're dealing with bounded convex P, we dont need the rays r_?
  matrix_t V_(num_verts, dim_verts);
  vector_t r_(num_verts);

  for (int i=0; i<num_verts; i++) {   
    V_.block(i,0,1,dim_verts) << P_vrep.V[i].transpose();
    r_(i) = 1.0;  // should be 1 if vertex, set to 0 if it's a ray. 
  }

  bool success = poly.setVrep(V_,r_);
  if (!success) {
    std::cout << "Failed to initialize Polytope in V rep." << std::endl;
  }

  auto hrep = poly.hrep();

  Polytope_H P_hrep;
  P_hrep.C = hrep.first;
  P_hrep.d = hrep.second;

  return P_hrep;
}

//////////////////////////////////////////////////////////

//convert polytope H rep to V rep
Polytope_V Graph::Hrep_to_Vrep(Polytope_H P_hrep) {
  
  Eigen::Polyhedron poly;
  
  bool success = poly.setHrep(P_hrep.C,P_hrep.d);
  if (!success) {
    std::cout << "Failed to initialize Polytope in H rep." << std::endl;
  }

  auto vrep = poly.vrep();
  
  // since we're dealing with bounded convex P, we dont need the rays r_.
  // Otherwise, will return rays for V-rep.
  matrix_t V_;
  //vector_t r_;
  V_ = vrep.first;
  //r_ = vrep.second; // returns 1 if vertex, 0 if ray.
  
  int num_verts = V_.rows();
  int dim = V_.cols();

  vector_array_t V;
  vector_t vert(dim);
  for (int i=0; i<num_verts; i++) {
    vert = V_.block(i,0,1,dim).transpose();
    V.push_back(vert);
  }

  Polytope_V P_vrep;
  P_vrep.V = V;
  
  return P_vrep;
}


