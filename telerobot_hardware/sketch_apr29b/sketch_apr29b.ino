void setup() {
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

  int bigFrontBack = analogRead(A1)-495; //главный рычаг, поворот вперед-назад
  int bigLeftRight = analogRead(A2)-510; //главный рычаг, поворот вправо-влево

  int plusMinus = analogRead(A4); //рычаг крутильный 

//КНОПКИ
  digitalWrite(9, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  
  bool lowLeftButton = digitalRead(8); //нижняя правая кнопка
  bool lowestButton = digitalRead(10); //самая нижняя кнопка
  bool third = digitalRead(2); //третья кнопка
  bool fourth = digitalRead(3); //четвертая кнопка

  delay(10);

//ДОП. КНОПКИ
  digitalWrite(9, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  
  bool five = digitalRead(10); //пятная кнопка
  bool six = digitalRead(8); //шестая кнопка
  

  delay(10);
  
//ВСПОМОГАТЕЛЬНЫЙ РЫЧАГ
  digitalWrite(9, LOW);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  
  bool smallBack = digitalRead(8); //вспомогательный рычаг, назад
  bool smallLeft = digitalRead(10); //вспомогательный рычаг, влево
  bool smallForw = digitalRead(3); //вспомогательный рычаг, вперед
  bool smallRight = digitalRead(2); //вспомогательный рычаг, вправо


  Serial.print(bigFrontBack);
  Serial.println("===");
  
  Serial.print(bigLeftRight);
  Serial.println("");

  Serial.print(smallBack);
  Serial.println("");

  Serial.print(smallLeft);
  Serial.println("");

  Serial.print(smallForw);
  Serial.println("");

  Serial.print(smallRight);
  Serial.println("");
  
  Serial.print(lowLeftButton);
  Serial.println("");

  Serial.print(lowestButton);
  Serial.println("");

  Serial.print(third);
  Serial.println("");

  Serial.print(fourth);
  Serial.println("");

  Serial.print(five);
  Serial.println("");
  
  Serial.print(six);
  Serial.println("");

  Serial.print(plusMinus);
  Serial.println("");
    
  delay(10);
  
}
