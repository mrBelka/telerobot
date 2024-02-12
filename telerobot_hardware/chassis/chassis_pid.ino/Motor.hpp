#pragma once

const int NUM_READ = 20;
const int IMP_PER_ROT = 2480;
const float WHEEL_CIRCUMSTANCE = 2*acos(-1);//0.25132741228;

class SPDMotor {
public:
  SPDMotor(int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2);

  void speed(int pwm);

  void setTarget(float setPoint)
  {
    m_setPoint = setPoint;
  }

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  float Job()
  {
    long curPos = getEncoderPosition(); 
    long dPos = curPos - m_lastPos;
    m_lastPos = curPos;
    float dt = (millis() - m_lastTime) / 1000.0; // in seconds
    m_lastTime = millis();
    float signal = (dPos * 1.0f / IMP_PER_ROT * WHEEL_CIRCUMSTANCE) / dt;
    //Serial.println(signal);
    float avr_signal = expRunningAverage(signal);
    //Serial.print(signal);
    //Serial.print(",");
    int controlAction = computePID(avr_signal, 32.4, 33, 10, dt, 0, 255);   // 810/25, 324/25, 506/25
    speed(controlAction);
    return avr_signal;
  }

    // коэффициент фильтрации, 0.0-1.0
  // бегущее среднее
  float expRunningAverage(float newVal) {
    filVal += (newVal - filVal) * k;
    return filVal;
  }

  // бегущее среднее арифметическое
  float runMiddleArifm(float newVal) 
  {  
    m_valArray[m_idx] = newVal;            
    if (++m_idx >= NUM_READ) 
      m_idx = 0;    
    float average = 0;                 
    for (int i = 0; i < NUM_READ; i++)
    {
      average += m_valArray[i];          
    }
    return (float)average / NUM_READ;
  }

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;
    int _speed;
    float m_setPoint = 0.0f;
    unsigned long m_lastTime = millis();
    long m_lastPos = 0;
    byte m_idx = 0;
    float m_valArray[NUM_READ];
    float m_integral = 0, m_prevErr = 0;

    float k = 0.1;
    float filVal = 0;

private:
    int computePID(float input, float kp, float ki, float kd, float dt, int minOut, int maxOut) 
    {
      float err = m_setPoint - input;
      m_integral = constrain(m_integral + (float)err * dt * ki, minOut, maxOut);
      float D = (err - m_prevErr) / dt;
      m_prevErr = err;
      return constrain(err * kp + m_integral + D * kd, minOut, maxOut);
    }

    
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