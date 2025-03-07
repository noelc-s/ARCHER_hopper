#include "ELMO_CANt4.h"

namespace Archer
{
  ELMO_CANt4::ELMO_CANt4() : status1_(ELMO_CANt4::STATUS::OFF),
                             status2_(ELMO_CANt4::STATUS::OFF),
                             status3_(ELMO_CANt4::STATUS::OFF),
                             status4_(ELMO_CANt4::STATUS::OFF),
                             baud_(CAN_BAUD),
                             can1_(),
                             msgIn_(),
                             msgOut_(),
                             recTimeout_(R_TIMEOUT)
  {
  }

  int32_t ELMO_CANt4::initB(void)
  {
    can1_.begin();
    can1_.setBaudRate(baud_);
    // check to see if MC is on
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
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          status4_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status4_ == ELMO_CANt4::STATUS::INIT)
      {
        break;
      }
    }
    if (status4_ == ELMO_CANt4::STATUS::INIT)
    {
      // Serial.println("Elmo has power, finding max current");
      setMaxC(IDX_);
      // Serial.println("Elmo has power, turing motor off");
      motorOff(IDX_);
      if (status4_ == ELMO_CANt4::STATUS::MOTOR_OFF)
      {
        rt = rt * 1;
      }
      else
      {
        // Serial.println("Something went wrong during MC off");
        rt = 0;
      }
    }
    else
    {
      // Serial.println("Something went wrong during MC init");
      rt = -1;
    }
    return rt;
  }

  int32_t ELMO_CANt4::initK(void)
  {
    can1_.begin();
    can1_.setBaudRate(baud_);
    // check to see if MC-K1 is on
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
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          status1_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status1_ == ELMO_CANt4::STATUS::INIT)
      {
        break;
      }
    }
    if (status1_ == ELMO_CANt4::STATUS::INIT)
    {
      Serial.println("Elmo1 has power, finding max current");
      setMaxC(IDX_);
      Serial.println("Elmo1 has power, turing motor1 off");
      motorOff(IDX_);
      if (status1_ == ELMO_CANt4::STATUS::MOTOR_OFF)
      {
        rt = rt * 1;
      }
      else
      {
        Serial.println("Something went wrong during MC1 off");
        rt = 0;
      }
    }
    else
    {
      Serial.println("Something went wrong during MC1 init");
      rt = -1;
    }
    // Initialize K2
    IDX_ = IDX_K2;
    msgOut_.id = 0x600 + IDX_;
    can1_.write(msgOut_);
    t0 = micros();
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          status2_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status2_ == ELMO_CANt4::STATUS::INIT)
      {
        break;
      }
    }
    if (status2_ == ELMO_CANt4::STATUS::INIT)
    {
      Serial.println("Elmo2 has power, turing motor2 off");
      motorOff(IDX_);
      if (status2_ == ELMO_CANt4::STATUS::MOTOR_OFF)
      {
        rt = rt * 2;
      }
      else
      {
        Serial.println("Something went wrong during MC2 off");
        rt = 0;
      }
    }
    else
    {
      Serial.println("Something went wrong during MC2 init");
      rt = -1;
    }
    // Initialize K3
    IDX_ = IDX_K3;
    msgOut_.id = 0x600 + IDX_;
    can1_.write(msgOut_);
    t0 = micros();
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          status3_ = ELMO_CANt4::STATUS::INIT;
          break;
        }
      }
      if (status3_ == ELMO_CANt4::STATUS::INIT)
      {
        break;
      }
    }
    if (status3_ == ELMO_CANt4::STATUS::INIT)
    {
      Serial.println("Elmo2 has power, turing motor3 off");
      motorOff(IDX_);
      if (status3_ == ELMO_CANt4::STATUS::MOTOR_OFF)
      {
        rt = rt * 3;
      }
      else
      {
        Serial.println("Something went wrong during MC3 off");
        rt = 0;
      }
    }
    else
    {
      Serial.println("Something went wrong during MC3 init");
      rt = -1;
    }
    return rt;
  }

  int32_t ELMO_CANt4::motorOff(uint16_t IDX_)
  {
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
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          switch (IDX_)
          {
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

  int32_t ELMO_CANt4::motorOn(uint16_t IDX_)
  {
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
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          switch (IDX_)
          {
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

  int32_t ELMO_CANt4::setMaxC(uint16_t IDX_)
  {
    bool write_rated_curr, write_rated_torque;
    bool read_rated_curr, read_rated_torque, read_max_curr, read_max_torque;

    // Write motor rated current
    // uint32_t rated_curr = static_cast<uint32_t>(0x26ac);  // 9.9A
    // uint16_t rated_curr = static_cast<uint16_t>(0x2af8); // 11A
    uint16_t rated_curr = static_cast<uint16_t>(0x3a98);  // 15A

    Serial.print("\nWriting motor rated current: ");
    Serial.print(rated_curr);
    Serial.println(" mA");
    write_rated_curr = write_data(IDX_, static_cast<uint16_t>(0x6075), 0, rated_curr);
    Serial.println(write_rated_curr ? "\tSuccess" : "\tFailure.");

    // Read motor rated current
    Serial.print("\nReading motor rated current: ");
    read_rated_curr = read_data(IDX_, static_cast<uint16_t>(0x6075), 0, maxC_);
    Serial.println(read_rated_curr ? String(maxC_) : "Failure.");

    // // Write motor rated torque
    // Serial.print("\nWriting motor rated torque: ");
    // Serial.print(rated_curr);
    // Serial.println(" mNm");
    // write_rated_torque = write_data(IDX_, static_cast<uint16_t>(0x6076), 0, rated_curr);
    // Serial.println(write_rated_torque ? "\t Success" : "Failure.");

    // // Read motor rated torque
    // Serial.print("\nReading motor rated torque: ");
    // uint32_t rated_torque;
    // read_rated_torque = read_data(IDX_, static_cast<uint16_t>(0x6076), 0, rated_torque);
    // Serial.println(read_rated_torque ? String(rated_torque) : "Failure.");

    // // Read maximal current
    // Serial.print("\nReading maximal current: ");
    // uint32_t max_curr_read_val;
    // read_max_curr = read_data(IDX_, static_cast<uint16_t>(0x6073), 0, max_curr_read_val);
    // Serial.println(read_max_curr ? String(max_curr_read_val) : "Failure.");

    // // Read maximal torque
    // Serial.print("\nReading maximal torque: ");
    // uint32_t max_torque;
    // read_max_torque = read_data(IDX_, static_cast<uint16_t>(0x6073), 0, max_torque);
    // Serial.println(read_max_torque ? String(max_torque) : "Failure.");

    // Serial.println(String(static_cast<uint16_t>(0x6075), HEX));

    // for (int i = 0; i < 30000; i++) {
    //   uint16_t port = static_cast<uint16_t>(0x1000 + i);
    //   uint32_t val;
    //   bool read_val = read_data(IDX_, port, 0, val);
    //   if (read_val && val != 100794368 && val != 4294967295 && val != 100728833) {
    //     Serial.print("Reading ");
    //     Serial.print(String(port, HEX));
    //     Serial.print(": ");
    //     Serial.println(val);
    //   }
    //   delay(1);
    // }

    return !(write_rated_curr && write_rated_torque && read_rated_curr && read_rated_torque && read_max_curr && read_max_torque);
  }

  uint32_t ELMO_CANt4::getMaxC(void)
  {
    return maxC_;
  }

  int32_t ELMO_CANt4::sendTC(float amps, uint16_t IDX_)
  {
    bool sendTC_succ = false;
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float nomVal = amps * 1000.0 * 1000.0 / (float)maxC_;
    n16.i = round(nomVal);
    // if ((n16.i > (int) maxC_)) {
    //   n16.i = (int) maxC_;
    // }
    // if ((n16.i< -(int) maxC_)) {
    //   n16.i = -(int) maxC_;
    // }
    if ((n16.i > 1000))
    {
      n16.i = 1000;
    }
    if ((n16.i < -1000))
    {
      n16.i = -1000;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
    uint32_t T0 = micros();
    while (micros() - T0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
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
          // return 1;
          sendTC_succ = true;
        }
      }
    }

    // // Report difference between commanded and actual currents
    // uint32_t cmd_curr;
    // bool read_cmd_curr = read_data(
    //     IDX_,
    //     static_cast<uint16_t>(0x6078),
    //     0,
    //     cmd_curr);
    // Serial.print("\nCommanded current: ");
    // Serial.print(nomVal);
    // Serial.print("; Actual current: ");
    // Serial.println(read_cmd_curr ? String((int16_t)cmd_curr) : "Failure.");

    // // Try reading 0x1001 - error codes
    // //    0x22A2, 0x22A3, 0x22A4 - temperatures

    // // Report error codes
    // uint32_t err_code;
    // bool read_err_code = read_data(
    //     IDX_,
    //     static_cast<uint16_t>(0x1001),
    //     0,
    //     err_code);
    // Serial.print("Error Code: ");
    // Serial.println(read_err_code ? String(err_code) : "Failure.");

    //     // Report error codes
    // uint32_t stat_code;
    // bool read_stat_code = read_data(
    //     IDX_,
    //     static_cast<uint16_t>(0x1002),
    //     0,
    //     stat_code);
    // Serial.print("Status Register: ");
    // Serial.println(read_stat_code ? String(stat_code, HEX) : "Failure.");

    // // Report temperatures
    // uint32_t temp1;
    // bool read_temp1 = read_data(
    //     IDX_,
    //     static_cast<uint16_t>(0x22a2),
    //     0,
    //     temp1);
    // // uint32_t temp2;
    // // bool read_temp2 = read_data(
    // //   IDX_,
    // //   static_cast<uint16_t>(0x22a3),
    // //   0,
    // //   temp2
    // // );
    // // uint32_t temp3;
    // // bool read_temp3 = read_data(
    // //   IDX_,
    // //   static_cast<uint16_t>(0x22a4),
    // //   0,
    // //   temp3
    // // );
    // Serial.print("Temp 1: ");
    // Serial.println(read_temp1 ? String(temp1) : "Failure.");
    // // Serial.print("  Temp 2: "); Serial.print(read_temp2 ? String(temp2) : "Failure.");
    // // Serial.print("  Temp 3: "); Serial.println(read_temp3 ? String(temp3) : "Failure.");

    // // Report error codes
    // uint32_t bus_voltage;
    // bool read_voltage = read_data(
    //     IDX_,
    //     static_cast<uint16_t>(0x6079),
    //     0,
    //     bus_voltage);
    // Serial.print("Voltage: ");
    // Serial.println(read_voltage ? String(bus_voltage) : "Failure.");

    return !sendTC_succ;
  }

  void ELMO_CANt4::cmdTC(float amps, uint16_t IDX_)
  {
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float nomVal = amps * 1000.0 * 1000.0 / (float)maxC_;
    n16.i = round(nomVal);
    if ((n16.i > 1000))
    {
      n16.i = 1000;
    }
    if ((n16.i < -1000))
    {
      n16.i = -1000;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    can1_.write(msgOut_);
  }

  int32_t ELMO_CANt4::getVel(int32_t &vel, uint16_t IDX_)
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

    can1_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros() - t0 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        if (msgIn_.id == IDX_ + (uint32_t)0x580)
        {
          num32_t n32;
          n32.c[0] = msgIn_.buf[4];
          n32.c[1] = msgIn_.buf[5];
          n32.c[2] = msgIn_.buf[6];
          n32.c[3] = msgIn_.buf[7];
          vel_ = n32.i;
          vel = vel_;
          // Serial.print("velocity is (in cnts/s): ");
          // Serial.println(vel_);
          return 1;
        }
      }
    }
    return 0;
  }

  bool ELMO_CANt4::write_data(uint32_t node_id, uint16_t index, uint8_t sub_index, uint32_t value)
  {
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + node_id;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;                // Write two bytes? what about 4 bytes&read
    msgOut_.buf[1] = index & 0xFF;        // Low byte of index
    msgOut_.buf[2] = (index >> 8) & 0xFF; // High byte of index
    msgOut_.buf[3] = sub_index;           // sub index
    msgOut_.buf[4] = value & 0xFF;        // Low byte of value
    msgOut_.buf[5] = (value >> 8) & 0xFF; // High byte of value
    msgOut_.buf[6] = (value >> 16) & 0xFF;
    msgOut_.buf[7] = (value >> 24) & 0xFF;

    // Serial.printf(
    //   "Write buffer:   %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //   msgOut_.buf[0], msgOut_.buf[1], msgOut_.buf[2], msgOut_.buf[3],
    //   msgOut_.buf[4], msgOut_.buf[5], msgOut_.buf[6], msgOut_.buf[7]
    // );

    can1_.write(msgOut_);
    uint32_t t00 = micros();
    while (micros() - t00 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        return true;
      }
    }
    return false;
  }

  bool ELMO_CANt4::read_data(uint32_t node_id, uint16_t index, uint8_t sub_index, uint32_t &data)
  {
    msgOut_.flags.extended = 0;
    msgOut_.id = 0x600 + node_id;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x40;                // Read
    msgOut_.buf[1] = index & 0xFF;        // Get low byte
    msgOut_.buf[2] = (index >> 8) & 0xFF; // Get high byte
    msgOut_.buf[3] = sub_index;           // Sub index
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    // Serial.printf(
    //   "ReadOut buffer: %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //   msgOut_.buf[0], msgOut_.buf[1], msgOut_.buf[2], msgOut_.buf[3],
    //   msgOut_.buf[4], msgOut_.buf[5], msgOut_.buf[6], msgOut_.buf[7]
    // );

    can1_.write(msgOut_);
    uint32_t t00 = micros();
    while (micros() - t00 < recTimeout_)
    {
      if (can1_.read(msgIn_))
      {
        num32_t n32;
        n32.c[0] = msgIn_.buf[4];
        n32.c[1] = msgIn_.buf[5];
        n32.c[2] = msgIn_.buf[6];
        n32.c[3] = msgIn_.buf[7];
        data = n32.ui;

        // Serial.printf(
        //   "ReadIn buffer:  %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //   msgIn_.buf[0], msgIn_.buf[1], msgIn_.buf[2], msgIn_.buf[3],
        //   msgIn_.buf[4], msgIn_.buf[5], msgIn_.buf[6], msgIn_.buf[7]
        // );
        return true;
      }
    }
    return false;
  }
}