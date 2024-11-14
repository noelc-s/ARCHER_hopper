#include "../inc/MPC.h"
#include <cassert>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

int MPC::solve(Hopper hopper, vector_t &sol, vector_3t &command, vector_2t &command_interp) {
    matrix_t x_bar(nx, p.N-1);
    matrix_t u_bar(nu, p.N-1);

    vector_t x0(21);
    vector_t x0_local(21);
    vector_t s0(20);
    x0 << hopper.state_.q, hopper.state_.v;
    x0_local = global2local(x0);
    s0 = qk_to_xik(x0_local,x0_local);
    vector_t log_x0(20);
    log_x0 = Log(x0_local);
    scalar_t t2i = time2impact(x0,p.heightOffset);
    if (hopper.state_.contact || t2i != t2i || t2i < 0) {
	    t2i = 0;
    }

    d_bar.setZero();
    x_bar.setZero();
    u_bar.setZero();
    bool first_impact = false;
    bool first_flight = false;
    int first_flight_index = 0;
    if (hopper.state_.contact) {
      first_impact = true;
    }
    scalar_t offset = 0;
    full_ref.setZero();
    elapsed_time.setZero();
    elapsed_time(0) = 0;

    static bool flip_started = false;
    static scalar_t flip_start_time = 100;
    if (flip_started == false && (abs(command(2) -1)<= 0.05 || abs(command(2) +1)<= 0.05) && t2i == 0) {
	flip_started = true;
	flip_start_time = hopper.state_.t;
    }

    static bool circle_started = false;
    static scalar_t circle_start_time = 100;

    if (abs(command(2)- 2) <= 0.05) {
      if (circle_started == false) {
	circle_started = true;
	circle_start_time = hopper.state_.t;
      } else {
        //command(0) = p.circle_amp*sin(4*3.14159*(hopper.state_.t-circle_start_time)/p.circle_freq);
        //command(1) = p.circle_amp*cos(2*3.14159*(hopper.state_.t-circle_start_time)/p.circle_freq)-p.circle_amp; 
	int corner = floor(4*(fmod(hopper.state_.t-circle_start_time, p.circle_freq))/p.circle_freq);
	switch(corner) {
		case 0: 
	    command(0) = 1;
	    command(1) = 1;
			break;
		case 1:
	    command(0) = 1;
	    command(1) = -1;
			break;
		case 2:
	    command(0) = -1;
	    command(1) = -1;
			break;
		case 3:
	    command(0) = -1;
	    command(1) = 1;
		break;
	}
      }
    } else {
      circle_started = false;
    }

    scalar_t alpha;
   if (flip_started) {
          alpha = command(2)*4*3.14;
	  if ((hopper.state_.t > (flip_start_time + .3)) && hopper.state_.contact == 1) {
            flip_started = false;
            command(2) = 0;
          }
   } else{
           alpha = 0;
   }
  
   if ((command.segment(0,2) - x0.segment(0,2)).norm()/(p.N*p.dt_flight) > p.max_vel) {
	command_interp = x0.segment(0,2) + (command.segment(0,2) - x0.segment(0,2))/((command.segment(0,2) - x0.segment(0,2))).norm()*p.N*p.dt_flight*p.max_vel;
   } else {
	   command_interp = command.segment(0,2);
   }

    for (int i = 0; i < p.N-1; i++){
	if (elapsed_time(i)-offset < t2i) {
          d_bar(i) = flight;
	  elapsed_time(i+1) = elapsed_time(i) + p.dt_flight;
	  if (!first_flight){
	    first_flight = true;
	    first_flight_index = i;
	  }
	} else {
	  if (t2i == 0 && elapsed_time(i)-offset+(hopper.state_.t-hopper.state_.last_impact_time)-t2i > p.groundDuration) {
	    if (!first_flight) {
		    d_bar(i) = ground_flight;
		    first_flight = true;
		    elapsed_time(i+1) = elapsed_time(i);
		    t2i = elapsed_time(i) + p.time_between_contacts;
		    first_impact = false;
		    first_flight = true;
		    first_flight_index = i;
	    } else{
	      d_bar(i) = flight;
	      elapsed_time(i+1) = elapsed_time(i) + p.dt_flight;
	    }
	  } else if (elapsed_time(i)-t2i-offset > p.groundDuration) {
	    if (!first_flight) {
		    d_bar(i) = ground_flight;
		    elapsed_time(i+1) = elapsed_time(i);
		    t2i = elapsed_time(i) + p.time_between_contacts;
		    first_impact = false;
		    first_flight = true;
		    first_flight_index = i;
	    } else {
	      d_bar(i) = flight;
	      elapsed_time(i+1) = elapsed_time(i) + p.dt_flight;
	    }
	  } else {
            if (first_impact == false) {
	      d_bar(i) = flight_ground;
	      elapsed_time(i+1) = elapsed_time(i);
	      first_impact = true;
	      first_flight = false;
	    } else{
	      d_bar(i) = ground;
	      elapsed_time(i+1) = elapsed_time(i) + p.dt_ground;
	    }
	  }
	}
	full_ref.segment(i*nx,2) << ((float) p.N-i)/p.N*x0.segment(0,2) + ((float) i)/p.N*command_interp.segment(0,2);
	full_ref.segment(i*nx+2,1) << p.hop_height;
	full_ref.segment(i*nx+3,3) << -log_x0.segment(3,3); // Hacky modification to cost to get orientation tracking back TODO
	if (first_flight) {
	  scalar_t t = elapsed_time(i) + hopper.state_.t - hopper.state_.last_flight_time;
	  scalar_t t_max = t2i + hopper.state_.t - hopper.state_.last_flight_time;
	  // Heuristic to deal with log
	  if (abs(alpha) > 0.1 && t < 0.285) {
	    full_ref(i*nx+4) = alpha*std::min(t/t_max,1.);
	  } else {
	    full_ref(i*nx+4) = -log_x0(4);
	  }
	}
    }
    //Terminal cost
    full_ref.segment((p.N-1)*nx,2) << command_interp.segment(0,2);
    full_ref.segment((p.N-1)*nx+2,1) << p.hop_height;
    full_ref.segment((p.N-1)*nx+3,3) << -log_x0.segment(3,3);
    if (!flip_started) {
      full_ref((p.N-1)*nx+4) = -log_x0(4);
    }
    x_bar.block(0,0,nx,1) << s0;
    for (int i = 1; i < p.N-1; i++){
	x_bar.block(0,i,nx,1) << oneStepPredict(hopper,x_bar.block(0,i-1,nx,1),u_bar.block(0,i-1,nu,1),elapsed_time(i+1)-elapsed_time(i),d_bar(i-1), x0_local);
    }
    f = -H*full_ref;

    for (int iter = 0; iter < p.SQP_iter; iter++) {
      LinearizeDynamics(hopper, x_bar, u_bar, d_bar, x0_local, elapsed_time);
      updateDynamicEquality(s0);
      auto status = solver.SetObjectiveVector(f);
      status = solver.UpdateConstraintMatrix(dynamics_A);
      status = solver.SetBounds(dynamics_b_lb, dynamics_b_ub);


      // solve the QP problem
      solver.Solve();
      sol = solver.primal_solution();
      if (iter < p.SQP_iter-1) {
        u_bar.block(0,0,nu,1) << sol.segment(p.N*nx,nu);
        for (int i = 1; i < p.N-1; i++){
          x_bar.block(0,i,nx,1) << sol.segment(i*nx,nx);
          u_bar.block(0,i,nu,1) << sol.segment(p.N*nx+i*nu,nu);
        }
      }
    }
    return 0;
}

scalar_t MPC::time2impact(vector_t x, scalar_t heightOffset) {
	scalar_t x0 = x(2)-heightOffset+x(7);
	scalar_t v0 = x(11+2);
	scalar_t g = 9.81;

	scalar_t t = (-v0-sqrt(pow(v0,2)+4*g*x0))/(-2*g);
	return t;
}


vector_t MPC::oneStepPredict(Hopper hopper, const vector_t xi, const vector_t tau,
                const float dt, const domain d, const vector_t q0) {
        matrix_t Ac(20,20);
        matrix_t Bc(20,4);
        matrix_t Cc(20,1);
        matrix_t Ad(20,20);
        matrix_t Bd(20,4);
        matrix_t Cd(20,1);
        vector_t s_k(20);
        vector_t s_kp1(20);
	hopper.DiscreteDynamics(xik_to_qk(xi,q0), tau.tail(4), d, dt, Ac, Bc, Cc, Ad, Bd, Cd,q0);
	s_k = xi;
        s_kp1 = Ad*s_k + Bd*tau.tail(4) + Cd;
        return s_kp1;
}

void MPC::LinearizeDynamics(Hopper hopper, matrix_t x_bar, matrix_t u_bar, Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar, const vector_t q0, const vector_t elapsed_time) {
    assertm(x_bar.rows() == nx, "Number of rows in x_bar not what expected");
    assertm(x_bar.cols() == p.N-1, "Number of cols in x_bar not what expected");
    assertm(u_bar.rows() == nu, "Number of rows in u_bar not what expected");
    assertm(u_bar.cols() == p.N-1, "Number of cols in u_bar not what expected");

    for (int i = 0; i < p.N-1; i++){
        hopper.DiscreteDynamics(xik_to_qk(x_bar.block(0,i,nx,1), q0), u_bar.block(0,i,nu,1),d_bar(i), elapsed_time(i+1)-elapsed_time(i), Ac_, Bc_, Cc_, Ad_, Bd_, Cd_,q0);
        Ac.block(0,i*nx,nx,nx) = Ac_;
        Bc.block(0,i*nu,nx,nu) = Bc_;
        Cc.block(0,i,nx,1) = Cc_;
        Ad.block(0,i*nx,nx,nx) = Ad_;
        Bd.block(0,i*nu,nx,nu) = Bd_;
        Cd.block(0,i,nx,1) = Cd_;
    }
}

void MPC::reset() {
    Ac.setZero();
    Bc.setZero();
    Cc.setZero();
    Ad.setZero();
    Bd.setZero();
    Cd.setZero();
    Ac_.setZero();
    Bc_.setZero();
    Cc_.setZero();
    Ad_.setZero();
    Bd_.setZero();
    Cd_.setZero();
    dynamics_A.setZero();
    dynamics_b_lb.setZero();
    dynamics_b_ub.setZero();
    H.setZero();
    f.setZero();
}

void MPC::buildDynamicEquality() {
    int offset = nx;
    for (int i = 0; i < p.N-1; i++) {
        for (int j = 0; j < nx; j++) {
            for (int k = 0; k < nx; k++) {
		//std::cout << "i,j,k" << i<<","<<j<<","<<k<<std::endl;
                dynamics_A.insert(offset+i * nx + j, i * nx + k) = 0;
            }
            for (int k = 0; k < nu; k++) {
                dynamics_A.insert(offset+i*nx+j,nx*p.N+i*nu+k) = 0;
            }
        }
    }
    for (int i = offset; i < nx*p.N; i++) {
        SparseIdentity.insert(i,i) = 1;
    }
    dynamics_A = SparseIdentity - dynamics_A;
    // for initial condition
    for (int i = 0; i < offset; i++) {
        dynamics_A.insert(i,i) = 1;
    }
    // set foot input to zero
    for (int i = 0; i < p.N-1; i++) {
      dynamics_A.insert(nx*p.N+i,nx*p.N+i*nu) = 1;
    }
    // torque_bounds
    for (int i = 0; i < p.N-1; i++) {
      for (int j = 0; j < 3; j++) {
        dynamics_A.insert(nx*p.N+p.N-1+i*3+j,nx*p.N+i*nu+j+1) = 1;
      }
    }
    // set foot input to zero
    for (int i = 0; i < p.N-1; i++) {
      dynamics_b_lb(nx*p.N+i) = 0;
      dynamics_b_ub(nx*p.N+i) = p.f_max;
    }
    // set torque limits
    for (int i = 0; i < p.N-1; i++) {
      for (int j = 0; j < 3; j++) {
        dynamics_b_lb(nx*p.N+p.N-1+i*3+j) = -p.tau_max;
        dynamics_b_ub(nx*p.N+p.N-1+i*3+j) = p.tau_max;
      }
    }
}

void MPC::updateDynamicEquality(vector_t x0) {
    int offset = nx;
    dynamics_b_lb.segment(0,nx) = x0;
    dynamics_b_ub.segment(0,nx) = x0;
    for (int i = 0; i < p.N-1; i++) {
        for (int j = 0; j < nx; j++) {
            for (int k = 0; k < nx; k++) {
                dynamics_A.coeffRef(offset+i * nx + j, i * nx + k) = -Ad(j, i * nx + k);
            }
            for (int k = 0; k < nu; k++) {
                dynamics_A.coeffRef(offset+i*nx+j,nx*p.N+i*nu+k) = -Bd(j,i*nu+k);
            }
        }
        dynamics_b_lb.segment(offset+i*nx,nx) << Cd.block(0,i,nx,1);
        dynamics_b_ub.segment(offset+i*nx,nx) << Cd.block(0,i,nx,1);
    }
}

void MPC::buildCost(){
    for (int i = 0; i < p.N; i++) {
        for (int j = 0; j < nx; j++) {
	  if (i == p.N-1) {
            H.insert(i*nx+j,i*nx+j) = p.terminalScaling*p.stateScaling(j);
	  } else {
            H.insert(i*nx+j,i*nx+j) = pow(p.discountFactor,i)*p.stateScaling(j);
	  }
        }
	if (i < p.N-1) {
          for (int j = 0; j < nu; j++) {
              H.insert(p.N*nx+i*nu+j,p.N*nx+i*nu+j) = pow(p.discountFactor,i)*p.inputScaling(j);
          }
	}
    }
    f.setZero();
}

