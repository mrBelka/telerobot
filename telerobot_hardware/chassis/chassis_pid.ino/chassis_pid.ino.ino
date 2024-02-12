#include <Servo.h>
#include <ServoSmooth.h>
#include <Encoder.h>
#include "Motor.hpp"

SPDMotor motorLF(18, 31, false, 12, 35, 34); // <- Encoder reversed to make +position measurement be forward.
SPDMotor motorRF(19, 38, true, 8, 37, 36); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor motorLR( 3, 49, false,  9, 42, 43); // <- Encoder reversed to make +position measurement be forward.
SPDMotor motorRR( 2, A1, true, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation

void setup()
{
  Serial.begin(115200);
  delay(50);
}

void loop() { 

  if(Serial.available()>0)
  {
      int data = Serial.read();
      if(data == 97)
      {
        motorLF.setTarget(6.f);
        motorRF.setTarget(6.f);
        motorLR.setTarget(6.f);
        motorRR.setTarget(6.f);
      }
      else
      {
        motorLF.setTarget(0.0f);
        motorRF.setTarget(0.0f);
        motorLR.setTarget(0.0f);
        motorRR.setTarget(0.0f);
      }
  }

  float lf_sig = motorLF.Job();
  float rf_sig = motorRF.Job();
  float lr_sig = motorLR.Job();
  float rr_sig = motorRR.Job();


  Serial.print(lf_sig);
  Serial.print(",");
  Serial.print(rf_sig);
  Serial.print(",");
  Serial.print(lr_sig);
  Serial.print(",");
  Serial.println(rr_sig);

  delay(45);
}