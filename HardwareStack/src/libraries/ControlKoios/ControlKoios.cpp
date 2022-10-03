#include "ControlKoios.h"

namespace Archer
{
  ControlKoios::ControlKoios(float Tmax){
    _Tmax = Tmax;
  }

  void ControlKoios::inputs(float &u1,float &u2,float &u3,float Y,float P,float R,float dY,float dP,float dR){
    float V_1,V_2,V_3,V1_1,V1_2,V1_3,V2_1,V2_2,V2_3,V3_1,V3_2,V3_3;
    float Q_0,Q_1,Q_2,Q_3,Q0_0,Q0_1,Q0_2,Q0_3,Qe_0,Qe_1,Qe_2,Qe_3;
    _Yaw   = Y*YY;
    _Pitch = P*PP;
    _Roll  = R*RR;
    dY = dY*DY;
    dP = dP*DP;
    dR = dR*DR;
    filterVel(dY,dP,dR);

    _v1 = cos(theta);
    _v2 = 0.0;
    _v3 = sin(theta);
    rotZYX(V1_1,V1_2,V1_3);
    _v1 = -0.5*cos(theta);
    _v2 = (sqrt(3.0)/2.0)*cos(theta);
    _v3 = sin(theta);
    rotZYX(V2_1,V2_2,V2_3);
    _v1 = -0.5*cos(theta);
    _v2 = -(sqrt(3.0)/2.0)*cos(theta);
    _v3 = sin(theta);
    rotZYX(V3_1,V3_2,V3_3);
    _e1 = _Yaw;
    _e2 = _Pitch;
    _e3 = _Roll;
    euler2quat(Q_0,Q_1,Q_2,Q_3);
    _e1 = Y_des;
    _e2 = P_des;
    _e3 = R_des;
    euler2quat(Q0_0,Q0_1,Q0_2,Q0_3);
    _p0 = Q0_0;
    _p1 = Q0_1;
    _p2 = Q0_2;
    _p3 = Q0_3;
    _q0 =  Q_0;
    _q1 = -Q_1;
    _q2 = -Q_2;
    _q3 = -Q_3;
    quatMult(Qe_0,Qe_1,Qe_2,Qe_3);
    _q0 = Qe_0;
    _q1 = Qe_1;
    _q2 = Qe_2;
    _q3 = Qe_3;
    quatAxis(V_1,V_2,V_3);
    float Np  = normK(Qe_1,Qe_2,Qe_3);
    float Nd  = normK(_dY,_dP,_dR);
    float D1p = dotK(V1_1,V1_2,V1_3,V_1,V_2,V_3);
    float D1d = dotK(V1_1,V1_2,V1_3,_dR,_dP,_dY);
    float D2p = dotK(V2_1,V2_2,V2_3,V_1,V_2,V_3);
    float D2d = dotK(V2_1,V2_2,V2_3,_dR,_dP,_dY);
    float D3p = dotK(V3_1,V3_2,V3_3,V_1,V_2,V_3);
    float D3d = dotK(V3_1,V3_2,V3_3,_dR,_dP,_dY);
    
    u1 = kp*Np*D1p + kd*Nd*D1d;
    u2 = kp*Np*D2p + kd*Nd*D2d;
    u3 = kp*Np*D3p + kd*Nd*D3d;

    saturate(u1,u2,u3);
  }

  void ControlKoios::rotZYX(float &V1,float &V2,float &V3){
    float c1 = cos(_Yaw);
    float c2 = cos(_Pitch);
    float c3 = cos(_Roll);
    float s1 = sin(_Yaw);
    float s2 = sin(_Pitch);
    float s3 = sin(_Roll);

    float R11 = c1*c2;
    float R12 = c1*s2*s3-c3*s1;
    float R13 = s1*s3+c1*c3*s2;
    float R21 = c2*s1;
    float R22 = c1*c3+s1*s2*s3;
    float R23 = c3*s1*s2-c1*s3;
    float R31 = -s2;
    float R32 = c2*s3;
    float R33 = c2*c3;

    V1 = R11*_v1 + R12*_v2 + R13*_v3;
    V2 = R21*_v1 + R22*_v2 + R23*_v3;
    V3 = R31*_v1 + R32*_v2 + R33*_v3;
  }

  void ControlKoios::euler2quat(float &Q0,float &Q1,float &Q2,float &Q3){
    _p0 = cos(_e1/2.0);
    _p1 = 0.0;
    _p2 = 0.0;
    _p3 = sin(_e1/2.0);
    _q0 = cos(_e2/2.0);
    _q1 = 0.0;
    _q2 = sin(_e2/2.0);
    _q3 = 0.0;

    float P0,P1,P2,P3;
    quatMult(P0,P1,P2,P3);

    _p0 = P0;
    _p1 = P1;
    _p2 = P2;
    _p3 = P3;
    _q0 = cos(_e3/2.0);
    _q1 = sin(_e3/2.0);
    _q2 = 0.0;
    _q3 = 0.0;

    quatMult(Q0,Q1,Q2,Q3);
  }

  void ControlKoios::quatMult(float &Q0,float &Q1,float &Q2,float &Q3){
    Q0 = _p0*_q0 - _p1*_q1 - _p2*_q2 - _p3*_q3;
    Q1 = _p0*_q1 - _p1*_q0 - _p2*_q3 - _p3*_q2;
    Q2 = _p0*_q2 - _p1*_q3 - _p2*_q0 - _p3*_q1;
    Q3 = _p0*_q3 - _p1*_q2 - _p2*_q1 - _p3*_q0;
  }

  void ControlKoios::quatAxis(float &V1,float &V2,float &V3){
    float val = _q1*_q1 + _q2*_q2 + _q3*_q3;
    float D = sqrt(val);

    if(D==0.0){
      V1 = 0;
      V2 = 0;
      V3 = 0;
    }
    else{
      V1 = _q1/D;
      V2 = _q2/D;
      V3 = _q3/D;
    }
  }

  float ControlKoios::normK(float v1,float v2,float v3){
    float tmp = v1*v1 + v2*v2 + v3*v3;
    float val = sqrt(tmp);
    return val;
  }

  float ControlKoios::dotK(float v1,float v2,float v3,float w1,float w2,float w3){
    float val  = v1*w1 + v2*w2 + v3*w3;
    return val;
  }

  void ControlKoios::saturate(float &u1,float &u2,float &u3){
    if(u1 > _Tmax){
      u1 = _Tmax;
    }
    else if(u1 < -_Tmax){
      u1 = -_Tmax;
    }
    if(u2 > _Tmax){
      u2 = _Tmax;
    }
    else if(u2 < -_Tmax){
      u2 = -_Tmax;
    }
    if(u3 > _Tmax){
      u3 = _Tmax;
    }
    else if(u3 < -_Tmax){
      u3 = -_Tmax;
    }
  }

  void ControlKoios::filterVel(float vY,float vP,float vR){
    if(vY>1.0){
      vY = 1.0; }
    else if(vY<-1.0){
      vY = -1.0; }
    if(vP>1.0){
      vP = 1.0; }
    else if(vP<-1.0){
      vP = -1.0; }
    if(vR>1.0){
      vR = 1.0; }
    else if(vR<-1.0){
      vR = -1.0; }
    _dY = _dY + (vY-_dY)*_fy;
    _dP = _dP + (vP-_dP)*_fp;
    _dR = _dR + (vR-_dR)*_fr;
  }

  void ControlKoios::setKp(int val){
    kp = (float)val;
  }

  void ControlKoios::setKd(int val){
    kd = (float)val;
  }

  void ControlKoios::setTx(int val){
    _Tmax = (float)val;
  }

  void ControlKoios::setYY(int val){
    YY = (float)val/100.0;
  }

  void ControlKoios::setDY(int val){
    DY = (float)val/100.0;
  }

  void ControlKoios::setPP(int val){
    PP = (float)val/100.0;
  }

  void ControlKoios::setDP(int val){
    DP = (float)val/100.0;
  }

  void ControlKoios::setRR(int val){
    RR = (float)val/100.0;
  }

  void ControlKoios::setDR(int val){
    DR = (float)val/100.0;
  }
}