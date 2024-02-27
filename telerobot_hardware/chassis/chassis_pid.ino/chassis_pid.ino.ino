#include <Servo.h>
#include <ServoSmooth.h>
#include <Encoder.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "Motor.hpp"

modbusDevice regBank;
modbusSlave slave;

SPDMotor motorLF(18, 31, false, 12, 35, 34); // <- Encoder reversed to make +position measurement be forward.
SPDMotor motorRF(19, 38, true, 8, 37, 36); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor motorLR( 3, 49, false,  9, 42, 43); // <- Encoder reversed to make +position measurement be forward.
SPDMotor motorRR( 2, A1, true, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation

ServoSmooth headRotateServo;
ServoSmooth headPitchServo;

void setup()
{
  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  regBank.add(30007);
  regBank.add(30008);
  regBank.add(40001);
  regBank.add(40002);
  regBank.add(40003);
  regBank.add(40004);
  regBank.add(40005);
  regBank.add(40006); //поворот головы

  regBank.set(40001, 3000);
  regBank.set(40002, 3000);
  regBank.set(40003, 3000);
  regBank.set(40004, 3000);
  regBank.set(40005, 0);
  regBank.set(40006, 0);

  slave._device = &regBank;  
  slave.setBaud(115200);

  headRotateServo.attach(44, 500, 2400);
  headPitchServo.attach(45, 90);
  headRotateServo.setCurrentDeg(90);
  headPitchServo.setCurrentDeg(90);
  headRotateServo.setSpeed(30);
  headPitchServo.setSpeed(30);
  headRotateServo.setTargetDeg(90);
  headPitchServo.setTargetDeg(90);

  delay(50);
}

void loop() { 

  float lf_sig = motorLF.Job();
  float rf_sig = motorRF.Job();
  float lr_sig = motorLR.Job();
  float rr_sig = motorRR.Job();

  long LFpos = lf_sig * 1000000;
  regBank.set(30001, (word) (LFpos / 32768) );
  regBank.set(30002, (word) (LFpos % 32768) );
  
  long RFpos = rf_sig * 1000000;
  regBank.set(30003, (word) (RFpos / 32768) );
  regBank.set(30004, (word) (RFpos % 32768) );
  
  long LRpos = lr_sig * 1000000;
  regBank.set(30005, (word) (LRpos / 32768) );
  regBank.set(30006, (word) (LRpos % 32768) );
  
  long RRpos = rr_sig * 1000000;
  regBank.set(30007, (word) (RRpos / 32768) );
  regBank.set(30008, (word) (RRpos % 32768) );

  // Set data
  motorLF.setTarget((int16_t)(regBank.get(40001)-3000)/100.f);
  motorRF.setTarget((int16_t)(regBank.get(40002)-3000)/100.f);
  motorLR.setTarget((int16_t)(regBank.get(40003)-3000)/100.f);
  motorRR.setTarget((int16_t)(regBank.get(40004)-3000)/100.f);

  headRotateServo.tick();
  headPitchServo.tick();
  headPitchServo.setTargetDeg((word) regBank.get(40005) + 90);
  headRotateServo.setTargetDeg((word)regBank.get(40006) + 90);

  slave.run();

  delay(50);
}