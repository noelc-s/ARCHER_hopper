///////////////////////////////////////////////////
////////////////////// Checks /////////////////////
///////////////////////////////////////////////////
//vector_t s_kp1(20);
//vector_t s(20);
//vector_t x_kp1(21);
//vector_t x_k(21);
//matrix_t Ac(20,20); Ac.setZero();
//matrix_t Bc(20,4); Bc.setZero();
//matrix_t Cc(20,1); Cc.setZero();
//matrix_t Ad(20,20); Ad.setZero();
//matrix_t Bd(20,4); Bd.setZero();
//matrix_t Cd(20,1); Cd.setZero();
//vector_t x(21);
//quat_t quat;
//manif::SO3<scalar_t> quat_;
//manif::SO3Tangent<scalar_t> xi;
//manif::SO3Tangent<scalar_t> xi_kp1;
//matrix_t J(6,10); J.setZero();
//vector_3t p_l, v_l, p_g, v_g;
//matrix_3t Rq;
//scalar_t qw,qx,qy,qz;
//vector_3t p;
//vector_3t q_dot;

//q << hopper.pos, hopper.quat.coeffs(), hopper.leg_pos, 0, 0, 0;
//v << hopper.vel, hopper.omega, hopper.leg_vel, hopper.wheel_vel;
//tau << 0, 0, 0, 0, 0, 0, hopper.torque;

// Does the Exp map cancel the Log map?
// Yes.
//quat_t quat(1,0,0,0);
//quat_t quat(0.7071,0,0,0.7071);
//quat.normalize();
//auto quat_ = manif::SO3<scalar_t>(hopper.quat);
//std::cout << "Quat: \t\t" << hopper.quat.coeffs().transpose() << std::endl;
//auto xi = quat_.log();
//std::cout << "log(Quat):\t " << xi << std::endl;
//std::cout << "exp(log(Quat)): " << xi.exp().quat().coeffs().transpose() << std::endl;
//std::cout << "------------------------------" << std::endl;

// Do the static dynamics make sense?
// Yes.
//quat = Eigen::Quaternion<scalar_t>(1,0,0,0);
//q << 0, 0, 1, quat.coeffs(), 0, 0, 0, 0;
//v << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//tau << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//d = ground;
//std::cout << "Static Dynamics: " << hopper.f(q,v,tau,d).transpose() << std::endl;
//matrix_t A(20,20);
//matrix_t B(20,4);
//matrix_t C(20,1);
//hopper.Df(q,v,tau,d,A,B,C);
//vector_t x_kp1(20);
//x_kp1 = hopper.oneStepPredict(q,v,tau,0.05,flight);


// Do the frame transformations cancel each other out?
// Yes. That does not mean that they are right though.
//p_g << q.segment(0,3);
//p_l << hopper.quat.inverse()._transformVector(p_g);
//q_local << p_l, hopper.quat.coeffs(),q.segment(7,4);
//qw = hopper.quat.w();
//qx = hopper.quat.x();
//qy = hopper.quat.y();
//qz = hopper.quat.z();
//Rq << pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy),
//   2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2),2*(qy*qz-qw*qx),
//   2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2);
//v_g = v.segment(0,3);
//v_l << Rq.transpose()*v_g + Hopper::cross(Rq.transpose()*p_g)*(1./2.*v.segment(3,3));
//v_local << v_l, v.segment(3,7);
//std::cout << "----------- Frame position transformations ---------" << std::endl;
//std::cout << "Global pos: " << p_g.transpose() << std::endl;
//std::cout << "Local pos: " << p_l.transpose() << std::endl;
//std::cout << "Global pos: " << hopper.quat._transformVector(p_l).transpose() << std::endl;
//std::cout << "----------- Frame velocity transformations ---------" << std::endl;
//std::cout << "Global vel: " << v_g.transpose() << std::endl;
//std::cout << "Local vel: " << v_l.transpose() << std::endl;
// Based on this: http://jamessjackson.com/lie_algebra_tutorial/09-lie_group_tables/
// Still do not know why it is positive and cross of the whole thing
//std::cout << "Global vel: " <<  (Rq*v_l - Rq*Hopper::cross(p_l)*(1./2.*v.segment(3,3))).transpose() << std::endl;
//std::cout << "State: " << state.transpose() << std::endl;
//std::cout << "d: " << d << std::endl;
//std::cout << "q: " << q.transpose() << std::endl;
//std::cout << "v: " << v.transpose() << std::endl;
//std::cout << "tau: " << tau.transpose() << std::endl;

// Do the implemented functions cancel each other out?
// Yes.
//vector_t x_local(21);
//vector_t x_global(21);
//x_global << hopper.q, hopper.v;
//x_local = MPC::global2local(x_global);
//std::cout << "x_global: " << x_global.transpose() << std::endl;
//std::cout << "x_local: " << x_local.transpose() << std::endl;
//std::cout << "x_global reset: " << MPC::local2global(x_local).transpose() << std::endl;
//std::cout << "-------------" << std::endl;

// Does the discrete update map make sense?
// Yes.
//std::cout << "x_minus: " << x_l.transpose() << std::endl;
//std::cout << "x_plus: " << hopper.delta_f(x_l.segment(0,11), x_l.segment(11,10), flight).transpose() << std::endl;

//FileHandleDebug << x.transpose().format(CSVFormat) << std::endl;
//X_kp1 << q_local, v_local;
//X_k << q, v;
//Quat = hopper.quat;
//Vector_t x_intermediate(21);
//For(int i = 0; i < predHorizon; i ++) {
//  x_kp1 = opt.oneStepPredict(hopper, sol_g.segment(0,11), sol_g.segment(11,10), tau, dt, d);
//  fileHandleDebug << x_kp1.transpose().format(CSVFormat) << std::endl;
//  //x_intermediate = MPC::local2global(x_kp1);
//  q << x_kp1.segment(0,11);
//  v << x_kp1.segment(11,10);
//}


// notes
// - Should make quat_to_xi function

// Printing Dynamics matrices to make sure they look right
//std::cout << "Ac: " << std::endl << opt.Ac.block(0,0,opt.nx,opt.nx) <<std::endl;
//std::cout << "--------------------------------------" << std::endl;
//std::cout << "Ad: " << std::endl << opt.Ad_ <<std::endl;
//std::cout << "--------------------------------------" << std::endl;
//std::cout << "Bc: " << std::endl << opt.Bc.block(0,0,opt.nx,opt.nu) <<std::endl;
//std::cout << "--------------------------------------" << std::endl;
//std::cout << "Bd: " << std::endl << opt.Bd_ <<std::endl;
//std::cout << "--------------------------------------" << std::endl;
//std::cout << "Cc: " << std::endl << opt.Cc.block(0,0,opt.nx,1) <<std::endl;
//std::cout << "--------------------------------------" << std::endl;
//std::cout << "Cd: " << std::endl << opt.Cd_ <<std::endl;
//std::cout << "--------------------------------------" << std::endl;


