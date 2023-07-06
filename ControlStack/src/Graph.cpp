#include "../inc/Graph.h"
#include<iostream>

using namespace std;

// check if point is in polytope
bool Graph::isInPolytope(vector_t x, Polytope P) {
  
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




