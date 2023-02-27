#include <Servo.h>
#include <ServoSmooth.h>
#include <Encoder.h>

#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>


#define nullptr NUL

modbusDevice regBank;
modbusSlave slave;

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

SPDMotor *motorLF = new SPDMotor(18, 31, false, 12, 35, 34); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, true, 8, 37, 36); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, false,  9, 42, 43); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, true, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation

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
  slave._device = &regBank;  
  slave.setBaud(115200);  
  
  headRotateServo.attach(44);
  headPitchServo.attach(45);
  headRotateServo.setCurrentDeg(90);
  headPitchServo.setCurrentDeg(90);
  
  headRotateServo.setSpeed(30);
  headPitchServo.setSpeed(30);
  
  headRotateServo.setTargetDeg(90);
  headPitchServo.setTargetDeg(90);

  regBank.set(40001, 255);
  regBank.set(40002, 255);
  regBank.set(40003, 255);
  regBank.set(40004, 255);
}

void loop() {  
 
  headRotateServo.tick();
  headPitchServo.tick();

  // Get data
  long LFpos = motorLF->getEncoderPosition();
  regBank.set(30001, (word) (LFpos / 65536) );
  regBank.set(30002, (word) (LFpos % 65536) );
  
  long RFpos = motorRF->getEncoderPosition();
  regBank.set(30003, (word) (RFpos / 65536) );
  regBank.set(30004, (word) (RFpos % 65536) );
  
  long LRpos = motorLR->getEncoderPosition();
  regBank.set(30005, (word) (LRpos / 65536) );
  regBank.set(30006, (word) (LRpos % 65536) );
  
  long RRpos = motorRR->getEncoderPosition();
  regBank.set(30007, (word) (RRpos / 65536) );
  regBank.set(30008, (word) (RRpos % 65536) );

  // Set data
  motorLF->speed(regBank.get(40001) - 255);
  motorRF->speed(regBank.get(40002) - 255);
  motorLR->speed(regBank.get(40003) - 255);
  motorRR->speed(regBank.get(40004) - 255);
  
  
  slave.run(); 
}
