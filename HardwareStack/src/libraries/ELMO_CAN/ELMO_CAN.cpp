#include "ELMO_CAN.h"

namespace Archer
{
  ELMO_CAN::ELMO_CAN(FlexCAN &port, const uint16_t &IDX, const uint32_t &recTimeout):
  status_(ELMO_CAN::STATUS::OFF),
  baud_(CAN_BAUD),
  port_(port),
  msgIn_(),
  msgOut_(),
  recTimeout_(recTimeout),
  IDX_(IDX)
  {}

  int32_t ELMO_CAN::init(void) {
    port_.begin(baud_);

    msgOut_.ext = 0;
    msgOut_.len = 8;

  //check to see if MC is on
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6041);
    msgOut_.ext = 0;
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

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
        if (msgIn_.id == IDX_ + 0x580) {
          status_ = ELMO_CAN::STATUS::INIT;
          Serial.println("Elmo has power, finding max current");
          break;
        }
      }
      if (status_ == ELMO_CAN::STATUS::INIT) {
        break;
      }
    }
    if (status_ == ELMO_CAN::STATUS::INIT) {
      //get max current
      getMaxC();
      //turn off motor
      motorOff();
    }
    if (status_ == ELMO_CAN::STATUS::MOTOR_OFF) {
      return 1;
    }
    else{
      Serial.println("Something went wrong during MC init");
      return 0;
    }
    // if (status_ != ELMO_CAN::STATUS::MOTOR_OFF) {
    //   Serial.println("Something went wrong during MC init");
    //   return 0;
    // }
  }

  int32_t ELMO_CAN::motorOff(void)
  {
    Serial.println("Now, we are shutting the motor down");
    num16_t n16;

    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.ext = 0;
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

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
        if (msgIn_.id == IDX_ + 0x580) {
          status_ = ELMO_CAN::STATUS::MOTOR_OFF;
          Serial.println("Motor is shutdown and ready to be turned on");
          return 1;
        }
      }
    }
    return 0;
  }

  int32_t ELMO_CAN::motorOn(void)
  {
    Serial.println("Now, we are starting the motor up");
    num16_t n16;

    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.ext = 0;
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

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
        if (msgIn_.id == IDX_ + 0x580) {
          status_ = ELMO_CAN::STATUS::MOTOR_ON;
          Serial.println("Motor is started and ready to be commanded");
          return 1;
        }
      }
    }
    return 0;
  }

  int32_t ELMO_CAN::getMaxC(void)
  {
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6075);
    msgOut_.ext = 0;
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

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
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

  int32_t ELMO_CAN::sendTC(float amps)
  {
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float nomVal = amps*1000.0*1000.0/(float)maxC_;
    n16.i = round(nomVal);
    if ((n16.i > (int) maxC_)) {
      n16.i = (int) maxC_;
    }
    if ((n16.i< -(int) maxC_)) {
      n16.i = -(int) maxC_;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    port_.write(msgOut_);
    uint32_t T0 = micros();
    while (micros()-T0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
        if (msgIn_.id == IDX_ + 0x580) {
          num32_t n32;
          n32.c[0] = msgIn_.buf[4];
          n32.c[1] = msgIn_.buf[5];
          n32.c[2] = msgIn_.buf[6];
          n32.c[3] = msgIn_.buf[7];
          int32_t retI = n32.i;
          Serial.print("response I is: ");
          Serial.println(retI);
          uint32_t retU = n32.ui;
          Serial.print("response U is: ");
          Serial.println(retU);
          return 1;
        }
        else{
          return 0;
        }
      }
    }
  }

  int32_t ELMO_CAN::getVel(void)
  {
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

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < recTimeout_) {
      while (port_.available()) {
        port_.read(msgIn_);
        if (msgIn_.id == IDX_ + 0x580) {
          num32_t n32;
          n32.c[0] = msgIn_.buf[4];
          n32.c[1] = msgIn_.buf[5];
          n32.c[2] = msgIn_.buf[6];
          n32.c[3] = msgIn_.buf[7];
          vel_ = n32.i;
          Serial.print("velocity is (in cnts/s): ");
          Serial.println(vel_);
          return 1;
        }
      }
    }
    return 0;
  }
}