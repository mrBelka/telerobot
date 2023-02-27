#include <SoftwareSerial.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>


#define nullptr NUL

SoftwareSerial RS485(3, 2); // RX, TX 

modbusDevice regBank;
modbusSlave slave;

const double tl431 = 2.43;
const double bat_4_calibr = 1.9402;
const double bat_3_calibr = 2.0634;
const double bat_2_calibr = 2.0766;
const double bat_1_calibr = 2;
const double charge_calibr = 2;
const double load_calibr = 1;

const int bat_1_controlPin = 4;
const int bat_2_controlPin = 5;
const int bat_3_controlPin = 6;
const int bat_4_controlPin = 7;
const int charge_controlPin = 8;
const int load_controlPin = 9;

const int load_measurePin = A5;
const int charge_measurePin = A4;
const int bat_4_measurePin = A3;
const int bat_3_measurePin = A2;
const int bat_2_measurePin = A1;
const int bat_1_measurePin = A0;


void ByPassOn(int controlPin)
{
  digitalWrite(controlPin, HIGH);
}

void ByPassOff(int controlPin)
{
  digitalWrite(controlPin, LOW);
}

void Charge(int controlPin, int p)
{
  analogWrite(controlPin, p);
}

void LoadOn(int controlPin)
{
  digitalWrite(controlPin, HIGH);
}

void LoadOff(int controlPin)
{
  digitalWrite(controlPin, LOW);
}

bool isDischarge1 = false;
bool isDischarge2 = false;
bool isDischarge3 = false;
bool isDischarge4 = false;

void setup() {

  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  slave._device = &regBank;  
  slave.setBaud(9600);  

  // Последовательный порт через RS-485
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  //RS485.begin(9600);
  
  analogReference(EXTERNAL);

  pinMode(bat_1_controlPin, OUTPUT);
  pinMode(bat_2_controlPin, OUTPUT);
  pinMode(bat_3_controlPin, OUTPUT);
  pinMode(bat_4_controlPin, OUTPUT);
  pinMode(charge_controlPin, OUTPUT);
  pinMode(load_controlPin, OUTPUT);
  ByPassOff(bat_1_controlPin);
  ByPassOff(bat_2_controlPin);
  ByPassOff(bat_3_controlPin);
  ByPassOff(bat_4_controlPin);
  Charge(charge_controlPin, 255);
  LoadOn(load_controlPin);
}

void loop() {
  double U4 = bat_4_calibr*analogRead(bat_4_measurePin)*tl431 / 1023;
  double U3 = bat_3_calibr*analogRead(bat_3_measurePin)*tl431 / 1023;
  double U2 = bat_2_calibr*analogRead(bat_2_measurePin)*tl431 / 1023;
  double U1 = bat_1_calibr*(analogRead(bat_1_measurePin)*tl431 / 1023); 
  double I_load = load_calibr*analogRead(load_measurePin)*tl431 / 1023;
  double I_charge = charge_calibr*analogRead(charge_measurePin)*tl431 / 1023;
  /*RS485.print("U4 ");RS485.println(U4);
  RS485.print("U3 ");RS485.println(U3);
  RS485.print("U2 ");RS485.println(U2);
  RS485.print("U1 ");RS485.println(U1);
  RS485.print("Lo ");RS485.println(I_load);
  RS485.print("Ch ");RS485.println(I_charge);*/

  regBank.set(30001, (word)(U1*1000) );
  regBank.set(30002, (word)(U2*1000) );
  regBank.set(30003, (word)(U3*1000)  );
  regBank.set(30004, (word)(U4*1000)  );
  regBank.set(30005, (word)(I_load*1000) );
  regBank.set(30006, (word)(I_charge*1000)  );
  slave.run(); 

  if(U4 > 3.6)
  {
    Charge(charge_controlPin, 0);
    ByPassOn(bat_4_controlPin);
    //RS485.println("Bat 4 discarge");
    //RS485.println("Charge STOP");
    isDischarge4 = true;
  }

  if(U3 > 3.6)
  {
    Charge(charge_controlPin, 0);
    ByPassOn(bat_3_controlPin);
    //RS485.println("Bat 3 discarge");
    //RS485.println("Charge STOP");
    isDischarge3 = true;
  }

  if(U2 > 3.6)
  {
    Charge(charge_controlPin, 0);
    ByPassOn(bat_2_controlPin);
    //RS485.println("Bat 2 discarge");
    //RS485.println("Charge STOP");
    isDischarge2 = true;
  }

  if(U1 > 3.6)
  {
    Charge(charge_controlPin, 0);
    ByPassOn(bat_1_controlPin);
    //RS485.println("Bat 1 discarge");
    //RS485.println("Charge STOP");
    isDischarge1 = true;
  }

  if(U4 < 3.35 && isDischarge4)
  {
    ByPassOff(bat_4_controlPin);
    isDischarge4 = false;
    Charge(charge_controlPin, 255);
    //RS485.println("Charge START");
  }

  if(U3 < 3.35 && isDischarge3)
  {
    ByPassOff(bat_3_controlPin);
    isDischarge3 = false;
    Charge(charge_controlPin, 255);
    //RS485.println("Charge START");
  }

  if(U2 < 3.35 && isDischarge2)
  {
    ByPassOff(bat_2_controlPin);
    isDischarge2 = false;
    Charge(charge_controlPin, 255);
    //RS485.println("Charge START");
  }

  if(U1 < 3.35 && isDischarge1)
  {
    ByPassOff(bat_1_controlPin);
    isDischarge1 = false;
    Charge(charge_controlPin, 255);
    //RS485.println("Charge START");
  }
}
