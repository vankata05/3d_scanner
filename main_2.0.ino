#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

LiquidCrystal_PCF8574 lcd(0x27);

#define sensor A0
#define center 12

int counter = 0;
float distance[200][200];
int pinA = 6;
int pinB = 7;
int pinALast;
int counter2 = 0;
int prev;
int turn = 0;
int menue = 0;
int menue_prev;
unsigned long currentTime;
unsigned long loopTime;
unsigned char encoder_A;
unsigned char encoder_B;
unsigned char encoder_A_prev=0;

float get_distance();
float get_avr();
bool scan_layer();
void move_stepper(int steps, int stepPin, int dirPin, bool dir);
void write_to_SD();

void setup() {
  // put your setup code here, to run once:
  int error;
  
  Serial.begin(9600);
  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode (6,INPUT);
  pinMode (7,INPUT);
  pinMode(8, INPUT_PULLUP);

  pinALast = digitalRead(pinA);
  currentTime = millis();
  loopTime = currentTime;

  SD.remove("MESH.TXT");
  Wire.begin();
  Wire.beginTransmission(0x27);  
  error = Wire.endTransmission();
  //Wire.setClock(10000);

  if(error != 0){
    Serial.println("Cannot find LCD");
  }else{
    lcd.begin(16, 2);
    lcd.setBacklight(255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("*** first line.");
    delay(1000);
    lcd.setCursor(0, 1);
    lcd.print("*** second line.");
    delay(1000);
    lcd.clear();  
  }    
  lcd.home();
  lcd.clear();
  lcd.print("Start scan");
}

void loop() {
  currentTime = millis();
  if(currentTime >= (loopTime + 5)){
  encoder_A = digitalRead(pinA);
  encoder_B = digitalRead(pinB);
  if((!encoder_A) && (encoder_A_prev)){
    
  if(encoder_B) {
    prev = counter2;
    counter2 --;
    if(counter2 < 0){
      counter2 = 36;
      prev = 37;
    }
  }else {
    prev = counter2;
    counter2 ++;
    if(counter2 > 36){
      counter2 = 0;
      prev = -1;    
    }
  }

  }
  menue_prev = menue;
  encoder_A_prev = encoder_A;
  loopTime = currentTime;
  }  
  if(prev > counter2){
    menue--;
    prev = counter2;
  }else if(prev < counter2){
    menue++;
    prev = counter2;
  }

  if(menue >= 2){
    menue = 0;    
  }else if(menue < 0){
    menue = 1;
  }


  switch (menue){
    case 0:  
      if(menue_prev != menue){ 
        lcd.home();
        lcd.clear();
        lcd.print("Start scan");
        delay(300);
      }
      if(digitalRead(8) == 0){
        bool gate = true;
        lcd.home();
        lcd.clear();  
        lcd.print("Scanning...");  
        counter = 0;
        while(gate && counter != 200){
          gate = scan_layer();
          counter++;
          Serial.println(counter);
        }        
        move_stepper(counter * 8, 3, 2, false);
        lcd.clear();
        lcd.print("Start scan");        
      }
      break;
    case 1:
      if(menue_prev != menue){ 
        lcd.home();
        lcd.clear();
        lcd.print("Save to SD");
        delay(300);
      }
      if(digitalRead(8) == 0){
        lcd.home();
        lcd.clear();
        lcd.print("Writing to SD...");
        write_to_SD();
        lcd.clear();
        lcd.print("Save to SD");
      }
      break;
  }
}

float get_distance(){
  float voltage = analogRead(sensor)*0.0048828125;
  float distance = 13*pow(voltage, -1);
  return distance;
}

float get_avr(){
  float distance, avr;
  avr = 0;
  for(int i = 0; i < 1000; i++){
    distance = get_distance();
    avr += distance;
  }
  if(avr/1000 <= 30 && avr/1000 >=5){
    return avr/1000;
  }else{
    return -1;
  }
}

void move_stepper(int steps, int stepPin, int dirPin, bool dir){
  if(dir == true){
    digitalWrite(dirPin, HIGH);
  }else{
    digitalWrite(dirPin, LOW);
  }
  for(int x = 0; x < steps; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}

bool scan_layer(){
  float avr;

  for(int i = 0; i != 200; i++){
    distance[counter][i] = get_avr();
    move_stepper(2, 4, 5, true);
  }

  avr = 0;
  for(int i = 0; i != 200; i++){
    avr += distance[counter][i];
  }

  if(avr/200 != -1 && counter <= 200){
    move_stepper(8, 3, 2, true);
    return true;
  }else{
    move_stepper(8, 3, 2, true);
    lcd.clear();
    lcd.print("Scan finished!");
    delay(1000);
    return false;
  }
}

void write_to_SD(){
  float x, y, z;
  File file;
  file = SD.open("mesh.txt", FILE_WRITE);

  if(!SD.begin(10)){
    lcd.clear();
    lcd.print("Initialization");
    lcd.setCursor(0, 1);
    lcd.print("failed!");
    delay(1000);
    return;
  }

  for(int a = 0; a < counter - 25; a++){
    for(int b = 0; b != 200; b++){
      if(distance[a][b] != -1){
        distance[a][b] = 14 - distance[a][b];
        z = a * 0.18;
        x = (cos(b*1.8*(3.141592 / 180.0)) * distance[a][b]);
        y = (sin(b*1.8*(3.141592 / 180.0)) * distance[a][b]);
        if(file){
          file.print(x);
          file.print(";");
          file.print(y);
          file.print(";");
          file.println(z);
        }else{
          lcd.clear();
          lcd.print("Can't open file!");
          return;
        }
      }
    }
  }

  file.close(); 
}
