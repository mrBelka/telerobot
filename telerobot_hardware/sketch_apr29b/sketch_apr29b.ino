#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

modbusDevice regBank;
modbusSlave slave;

void setup() {

  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  slave._device = &regBank;  
  slave.setBaud(115200); 
  
  Serial.begin(115200);

  pinMode(9, OUTPUT);

  digitalWrite(9, HIGH);
  pinMode(8, INPUT);

  pinMode(10, INPUT);
  pinMode(3, INPUT);
  pinMode(2, INPUT);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
 }

void loop() {

  int mlr = analogRead(A1)-495; //главный рычаг, поворот вперед-назад
  int mfb = analogRead(A2)-510; //главный рычаг, поворот вправо-влево
  int asb = analogRead(A4)-20; //рычаг крутильный 

  digitalWrite(9, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  
  bool five = digitalRead(10); //пятная кнопка
  bool six = digitalRead(8); //шестая кнопка

  digitalWrite(9, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  
  bool two = digitalRead(8); //нижняя правая кнопка
  bool one = digitalRead(10); //самая нижняя кнопка
  bool three = digitalRead(2); //третья кнопка
  bool four = digitalRead(3); //четвертая кнопка

  delay(10);
  
  digitalWrite(9, LOW);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  
  bool m_b = digitalRead(8); //вспомогательный рычаг, назад
  bool m_l = digitalRead(10); //вспомогательный рычаг, влево
  bool m_f = digitalRead(3); //вспомогательный рычаг, вперед
  bool m_r = digitalRead(2); //вспомогательный рычаг, вправо

  regBank.set(30001, (word)(mfb) );
  regBank.set(30002, (word)(mlr) );
  regBank.set(30003, (word)(m_f)  );
  regBank.set(30004, (word)(m_r)  );
  regBank.set(30005, (word)(one) );
  regBank.set(30006, (word)(two)  );
  slave.run(); 

    
  delay(10);
  
}
