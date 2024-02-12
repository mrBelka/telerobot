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

  regBank.set(40001, 3000);
  regBank.set(40002, 3000);
  regBank.set(40003, 3000);
  regBank.set(40004, 3000);

  slave._device = &regBank;  
  slave.setBaud(115200);  

  //Serial.begin(115200);
  delay(50);
}

void loop() { 

 /* if(Serial.available()>0)
  {
      int data = Serial.read();
      if(data == 97)
      {
        motorLF.setTarget(-6.f);
        motorRF.setTarget(-6.f);
        motorLR.setTarget(-6.f);
        motorRR.setTarget(-6.f);
      }
      else
      {
        motorLF.setTarget(0.0f);
        motorRF.setTarget(0.0f);
        motorLR.setTarget(0.0f);
        motorRR.setTarget(0.0f);
      }
  }*/

  float lf_sig = motorLF.Job();
  float rf_sig = motorRF.Job();
  float lr_sig = motorLR.Job();
  float rr_sig = motorRR.Job();


  /*Serial.print(lf_sig);
  Serial.print(",");
  Serial.print(rf_sig);
  Serial.print(",");
  Serial.print(lr_sig);
  Serial.print(",");
  Serial.println(rr_sig);*/

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
  motorLF.setTarget((regBank.get(40001) - 3000)/100.f);
  motorRF.setTarget((regBank.get(40002) - 3000)/100.f);
  motorLR.setTarget((regBank.get(40003) - 3000)/100.f);
  motorRR.setTarget((regBank.get(40004) - 3000)/100.f);

  slave.run();

  delay(45);
}