#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

// Arduino nano Atmega168

modbusDevice regBank;
modbusSlave slave;

const int mainForwBackPin = A2;
const int mainLeftRightPin = A1;
const int mainRotationalPin = A0;
const int auxRotationalPin = A4;

void setup() 
{
  // Настройка Modbus
  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  regBank.add(30007);
  regBank.add(30008);
  regBank.add(30009);
  regBank.add(30010);
  regBank.add(30011);
  regBank.add(30012);
  regBank.add(30013);
  //regBank.add(30014); // Не работает, ошибка MODBUS
  slave._device = &regBank;  
  slave.setBaud(115200); 

  // Настройка входов
  // Вспомогательный рычаг
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  pinMode(8, INPUT);
  pinMode(10, INPUT);
  pinMode(3, INPUT);
  pinMode(2, INPUT);

  // Кнопки
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
}

void loop() 
{
  // Главный рычаг (вперед-назад)
  int mainForwBackValue = analogRead(mainForwBackPin);

  // Главный рычаг (влево-вправо)
  int mainLeftRightValue = analogRead(mainLeftRightPin);

  // Главный рычаг (вращение)
  int mainRotationalValue = analogRead(mainRotationalPin);

  // Рычаг крутильный (+/-)
  int rotationalValue = analogRead(auxRotationalPin);

  digitalWrite(9, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  
  bool butFiveValue = digitalRead(10); //пятная кнопка
  bool butSixValue = digitalRead(8); //шестая кнопка

  digitalWrite(9, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  
  bool butTwoValue = digitalRead(8); //нижняя правая кнопка
  bool butOneValue = digitalRead(10); //самая нижняя кнопка
  bool butThreeValue = digitalRead(2); //третья кнопка
  bool butFourValue = digitalRead(3); //четвертая кнопка

  delay(10);
  
  // Вспомогательный рычаг
  digitalWrite(9, LOW);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  
  bool auxForwValue = digitalRead(3);   //вспомогательный рычаг, вперед
  bool auxBackValue = digitalRead(8);   //вспомогательный рычаг, назад
  bool auxLeftValue = digitalRead(10);  //вспомогательный рычаг, влево
  bool auxRightValue = digitalRead(2);  //вспомогательный рычаг, вправо

  // Помещаем данные в регистры
  regBank.set(30001, mainForwBackValue);
  regBank.set(30002, mainLeftRightValue);
  regBank.set(30003, mainRotationalValue);
  regBank.set(30004, rotationalValue);
  regBank.set(30005, auxForwValue);
  regBank.set(30006, auxBackValue);
  regBank.set(30007, auxLeftValue);
  regBank.set(30008, auxRightValue);
  regBank.set(30009, butOneValue);
  regBank.set(30010, butTwoValue);
  regBank.set(30011, butThreeValue);
  regBank.set(30012, butFourValue);
  regBank.set(30013, butFiveValue);
  //regBank.set(30014, butSixValue);  // Не работает, ошибка MODBUS

  // Опрос
  slave.run(); 

  delay(10);  
}
