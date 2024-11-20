#include "ELMO_CANt4.h"

namespace Archer
{
  ELMO_CANt4::ELMO_CANt4():
  status1_(ELMO_CANt4::STATUS::OFF),
  status2_(ELMO_CANt4::STATUS::OFF),
  status3_(ELMO_CANt4::STATUS::OFF),
  status4_(ELMO_CANt4::STATUS::OFF),
  baud_(CAN_BAUD),
  can1_(),
  msgIn_(),
  msgOut_(),
  recTimeout_(R_TIMEOUT)
  {}

  int32_t ELMO_CANt4::initB(void){
    can1_.begin();
    can1_.setBaudRate(baud_);
  //check to see if MC is on
    uint16_t IDX_ = IDX_BIA;
    int32_t rt = 1;
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6041);
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x48; // Can this just be 0x40 ?
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          status4_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status4_ == ELMO_CANt4::STATUS::INIT) {
        break;
      }
    }
    if (status4_ == ELMO_CANt4::STATUS::INIT) {
      // Serial.println("Elmo has power, finding max current");
      setMaxC(IDX_);
      // Serial.println("Elmo has power, turing motor off");
      motorOff(IDX_);
      if (status4_ == ELMO_CANt4::STATUS::MOTOR_OFF) {
        rt = rt*1;
      }
      else{
        // Serial.println("Something went wrong during MC off");
        rt = 0;
      }
    }
    else{
      // Serial.println("Something went wrong during MC init");
      rt = -1;
    }
    return rt;
  }

  int32_t ELMO_CANt4::initK(void){
    can1_.begin();
    can1_.setBaudRate(baud_);
  //check to see if MC-K1 is on
    uint16_t IDX_ = IDX_K1;
    int32_t rt = 1;
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6041);
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x48; // Can this just be 0x40 ?
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;
    // Initialize K1
    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          status1_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status1_ == ELMO_CANt4::STATUS::INIT) {
        break;
      }
    }
    if (status1_ == ELMO_CANt4::STATUS::INIT) {
      // Serial.println("Elmo1 has power, finding max current");
      setMaxC(IDX_);
      // Serial.println("Elmo1 has power, turing motor1 off");
      motorOff(IDX_);
      if (status1_ == ELMO_CANt4::STATUS::MOTOR_OFF) {
        rt = rt * 1;
      }
      else{
        // Serial.println("Something went wrong during MC1 off");
        rt = 0;
      }
    }
    else{
      // Serial.println("Something went wrong during MC1 init");
      rt = -1;
    }
    // Initialize K2
    IDX_ = IDX_K2;
    msgOut_.id = 0x600 + IDX_;
    can1_.write(msgOut_);
    t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          status2_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status2_ == ELMO_CANt4::STATUS::INIT) {
        break;
      }
    }
    if (status2_ == ELMO_CANt4::STATUS::INIT) {
      // Serial.println("Elmo2 has power, turing motor2 off");
      motorOff(IDX_);
      if (status2_ == ELMO_CANt4::STATUS::MOTOR_OFF) {
        rt = rt * 2;
      }
      else{
        // Serial.println("Something went wrong during MC2 off");
        rt = 0;
      }
    }
    else{
      // Serial.println("Something went wrong during MC2 init");
      rt = -1;
    }
    // Initialize K3
    IDX_ = IDX_K3;
    msgOut_.id = 0x600 + IDX_;
    can1_.write(msgOut_);
    t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          status3_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status3_ == ELMO_CANt4::STATUS::INIT) {
        break;
      }
    }
    if (status3_ == ELMO_CANt4::STATUS::INIT) {
      // Serial.println("Elmo2 has power, turing motor2 off");
      motorOff(IDX_);
      if (status3_ == ELMO_CANt4::STATUS::MOTOR_OFF) {
        rt = rt * 3;
      }
      else{
        // Serial.println("Something went wrong during MC2 off");
        rt = 0;
      }
    }
    else{
      // Serial.println("Something went wrong during MC2 init");
      rt = -1;
    }
    return rt;
  }

  int32_t ELMO_CANt4::motorOff(uint16_t IDX_){
    // Serial.println("Now, we are shutting the motor down");
    num16_t n16;

    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B; // Can this just be 0x20 ?
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 6;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          switch(IDX_){
            case 1:
              status1_ = ELMO_CANt4::STATUS::MOTOR_OFF;
              break;
            case 2:
              status2_ = ELMO_CANt4::STATUS::MOTOR_OFF;
              break;
            case 3:
              status3_ = ELMO_CANt4::STATUS::MOTOR_OFF;
              break;
            case 4:
              status4_ = ELMO_CANt4::STATUS::MOTOR_OFF;
              break;
          }
          // Serial.println("Motor is shutdown and ready to be turned on");
          return 1;
        }
      }
    }
    return 0;
  }

  int32_t ELMO_CANt4::motorOn(uint16_t IDX_){
    // Serial.println("Now, we are starting the motor up");
    num16_t n16;

    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B; // Can this just be 0x20 ?
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 15;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          switch(IDX_){
            case 1:
              status1_ = ELMO_CANt4::STATUS::MOTOR_ON;
              break;
            case 2:
              status2_ = ELMO_CANt4::STATUS::MOTOR_ON;
              break;
            case 3:
              status3_ = ELMO_CANt4::STATUS::MOTOR_ON;
              break;
            case 4:
              status4_ = ELMO_CANt4::STATUS::MOTOR_ON;
              break;
          }
          // Serial.println("Motor is started and ready to be commanded");
          return 1;
        }
      }
    }
    return 0;
  }

  int32_t ELMO_CANt4::setMaxC(uint16_t IDX_){
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6075);

    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x40;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          num32_t n32;
          n32.c[0] = msgIn_.buf[4];
          n32.c[1] = msgIn_.buf[5];
          n32.c[2] = msgIn_.buf[6];
          n32.c[3] = msgIn_.buf[7];
          maxC_ = n32.ui;
          Serial.print("The max current for this motor is (in mA): ");
          Serial.println(n32.ui);
          return 1;
        }
      }
    }
    return 0;
  }

  uint32_t ELMO_CANt4::getMaxC(void){
    return maxC_;
  }

  int32_t ELMO_CANt4::sendTC(float amps, uint16_t IDX_){
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float nomVal = amps*1000.0*1000.0/(float)maxC_;
    n16.i = round(nomVal);
    // if ((n16.i > (int) maxC_)) {
    //   n16.i = (int) maxC_;
    // }
    // if ((n16.i< -(int) maxC_)) {
    //   n16.i = -(int) maxC_;
    // }
    if ((n16.i > 1000)) {
      n16.i = 1000;
    }
    if ((n16.i < -1000)) {
      n16.i = -1000;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t T0 = micros();
    while (micros()-T0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          // num32_t n32;
          // n32.c[0] = msgIn_.buf[4];
          // n32.c[1] = msgIn_.buf[5];
          // n32.c[2] = msgIn_.buf[6];
          // n32.c[3] = msgIn_.buf[7];
          // int32_t retI = n32.i;
          // Serial.print("response I is: ");
          // Serial.println(retI);
          // uint32_t retU = n32.ui;
          // Serial.print("response U is: ");
          // Serial.println(retU);
          return 1;
        }
      }
    }
    return 0;
  }

  void ELMO_CANt4::cmdTC(float amps, uint16_t IDX_){
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float nomVal = amps*1000.0*1000.0/(float)maxC_;
    n16.i = round(nomVal);
    if ((n16.i > 1000)) {
      n16.i = 1000;
    }
    if ((n16.i < -1000)) {
      n16.i = -1000;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
  }

  int32_t ELMO_CANt4::getVel(int32_t &vel, uint16_t IDX_){
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6069);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x40;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < recTimeout_) {
      if (can1_.read(msgIn_)) {
        if (msgIn_.id == IDX_ + 0x580) {
          num32_t n32;
          n32.c[0] = msgIn_.buf[4];
          n32.c[1] = msgIn_.buf[5];
          n32.c[2] = msgIn_.buf[6];
          n32.c[3] = msgIn_.buf[7];
          vel_ = n32.i;
          vel  = vel_;
          // Serial.print("velocity is (in cnts/s): ");
          // Serial.println(vel_);
          return 1;
        }
      }
    }
    return 0;
  }
}