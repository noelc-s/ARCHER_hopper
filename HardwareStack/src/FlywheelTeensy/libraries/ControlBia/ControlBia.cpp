#include "ControlBia.h"

namespace Archer
{
  ControlBia::ControlBia(float Tmax){
    _Tmax = Tmax;
    _I = 0.0;
  }

  void ControlBia::resetI(){
    _I = 0.0;
  }

  void ControlBia::getRB0(float &val){
    val = _rb0;
  }

  void ControlBia::inputPID0(float &u,float d0,float rb,float wb,float xf){
    float xb = -(rb-_rb0)*_rad/_gr;
    float vb = -wb*_rad/_gr;
    float eb = xb - (d0*_st);
    float u0 = -((d0*_st)*_ks*_rad)/(_gr*_kf);
    _I = _I + kI*eb;
    if(_I > _Imax){
      _I = _Imax;
    }
    else if(_I < -_Imax){
      _I = -_Imax;
    }

    u = u0 + kP*eb + kD*vb + _I;
    saturate(u);
  }

  void ControlBia::testPID0(float &u0,float &up,float &ud,float &ui,float d0,float rb,float wb){
    float xb = -(rb-_rb0)*_rad/_gr;
    float vb = -wb*_rad/_gr;
    float eb = xb - (d0*_st);
    _I = _I + kI*eb;
    if(_I > _Imax){
      _I = _Imax;
    }
    else if(_I < -_Imax){
      _I = -_Imax;
    }
    u0 = -((d0*_st)*_ks*_rad)/(_gr*_kf);
    up = kP*eb;
    ud = kD*vb;
    ui = _I;
  }

  void ControlBia::testPDb(float &up,float &ud,float rb, float wb){
    float r = -(rb-_rb0);
    up = kPb*r;
    ud = kDb*(-wb);
  }

  void ControlBia::testPDf(float &u0,float &up,float &ud,float d0,float xf, float vf){
    float x = xf/1000.0;
    float v = vf/1000.0;
    float e = x - d0;
    u0 = -(d0*_ks*_rad)/(_gr*_kf);
    up = kP*e;
    ud = kD*v;
  }

  void ControlBia::testU0(float &up,float &ud,float d0,float xf, float vf){
    float x = xf/1000.0;
    float v = vf/1000.0;
    float e = x - d0;
    up = kP*e;
    ud = kD*v;
  }

  void ControlBia::logZero(float val,int i){
    if(i==1){
      _rb1 = val;
    }
    if(i==2){
      _rb2 = val;
      _rb0 = (_rb1+_rb2)/2.0;
    }
  }

  void ControlBia::saturate(float &u){
    if(u > _Tmax){
      u = _Tmax;
    }
    else if(u < -_Tmax){
      u = -_Tmax;
    }
  }

  void ControlBia::setTx(int val){
    _Tmax = (float)val;
  }
}