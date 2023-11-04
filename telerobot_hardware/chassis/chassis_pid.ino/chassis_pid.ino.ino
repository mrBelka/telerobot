#include <Servo.h>
#include <ServoSmooth.h>
#include <Encoder.h>
#include <PIDtuner2.h>

#define nullptr NUL

PIDtuner2 tuner;

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

unsigned long last_time;
long last_pos = 0;

void setup()
{
  Serial.begin(115200);
  tuner.setParameters(NORMAL, 0, 255, 1000, 0.1, 10);

  last_time = millis();
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void loop() {  
  // Get data
  long LFpos = motorLF->getEncoderPosition();
  
  
  long RFpos = motorRF->getEncoderPosition();
  
  long LRpos = motorLR->getEncoderPosition();
  
  long RRpos = motorRR->getEncoderPosition();

  long dpos = LFpos - last_pos;
  last_pos = LFpos;
  float dt = (millis() - last_time)/1000.0;
  last_time = millis();
  float signal = (dpos / 2480.0 * 0.25132741228718345) / dt;
  /*Serial.println("====");
  Serial.println(dpos);
  Serial.println(LFpos);
  Serial.println(last_pos);*/
  Serial.println(signal);
  Serial.print(computePID(signal, 0.35, 2053.39, 58491.40, 11.18, dt, 0, 255));
  //tuner.setInput(signal);
  //tuner.compute();

  //tuner.getOutput();
  //analogWrite(..., getOutput());
  motorLF->speed(computePID(signal, 0.35, 2053.39, 58491.40, 11.18, dt, 0, 255));
  //tuner.debugText();
  //tuner.debugPlot();

  // Set data
  //motorLF->speed(50); // -255..255
  motorRF->speed(0);
  motorLR->speed(0);
  motorRR->speed(0);

  delayMicroseconds(10000);
}