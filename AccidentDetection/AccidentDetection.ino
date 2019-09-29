//test change 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define NBEEPDEBUG
#define NGYRODEBUG
#define NALARMDEBUG
//#define NACCELDEBUG
//#define NGPSDEBUG
#define NBTDEBUG
#define NBLUETOOTH

#define GYROBUFFERSIZE 15
#define GYROINTERVAL 250

template <typename T>
struct Coords {
  T X;
  T Y;
  T Z;
};

Coords<float>& operator+=(Coords<float> &a, const Coords<float> &b) {
  a.X += b.X;
  a.Y += b.Y;
  a.Z += b.Z;
  return a;
}

Coords<float> operator/(Coords<float> a, const int &b) {
  a.X /= b;
  a.Y /= b;
  a.Z /= b;
  return a;
}

struct AlarmVars {
  int Repetition, OnInterval, OffInterval;
  int * ptrCurInterval;
  bool flagEnabled, flagOngoing;
  byte Sensitivity, gpsAvailability = false;
  AlarmVars() : Repetition{0}, OnInterval{0}, OffInterval{0}, ptrCurInterval{nullptr},
                flagEnabled{false}, flagOngoing{false}, Sensitivity{0}  { }
};

template <typename T>
class Buffer {
  private:
  T m_data[GYROBUFFERSIZE];
  int counterElements;
  void shift() {
    for (int i = 0; i < GYROBUFFERSIZE - 1; i++) {
      m_data[i] = m_data[i+1];
    }
  }
  
  public:
  Buffer() : m_data{}, counterElements{0} { }
  
  T& operator[](int i) {
    return m_data[i];
  }
  
  void insert(T entry) {
    if (counterElements < GYROBUFFERSIZE) {
      m_data[counterElements] = entry; 
      counterElements++;
    } else {
      shift();
      m_data[counterElements-1] = entry;
    }
  }
  
  void flush() {
    counterElements = 0;
    for (int i = 0; i < GYROBUFFERSIZE; i++) {
        m_data[i] = {};
      }
    }
  int getCount() {
    return counterElements;
  }
};

Coords<float> movementAlarm(); //remove this and get a free compile error
  

Coords<int> accel, gyro, pAccel;
Coords<float> gForce, rot, compensateVal;
AlarmVars alarm{};
Buffer<Coords<float>> gyroBuffer{};

#define S808 Serial3
#define BT Serial2

String textMessage = "", mainNumber = "+639503610262";
unsigned long previousMillisAlarm = 0, previousMillisGyro = 0, previousMillisResponse = 0, previousAccident = 0;
double Lat = 0.0, Lon = 0.0;
const byte buzz = 13, btn = 4;
int gpsData[6], replyFlag;
bool gpsWarn = false, location_warn = false, parkAlarmEnabled = false, isAccident = false, gpsAvailability = false, tilted = false;
char str1[32], str2[32];
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.print("BOOTING UP . . .");
  lcd.setCursor(0,1);
  lcd.print("Keep Safe Always");
  pinMode(btn, INPUT_PULLUP);
  pinMode(buzz, OUTPUT);
  Serial.begin(9600);
  BT.begin(9600);
  Wire.begin();
  delay(1000);
  S808.begin(9600);
  S808.print("AT+CGPSPWR=1\r\n");
  S808.print("AT+ CMGF=1\r");
  S808.print("AT+CNMI=2,2,0,0,0\r");
  S808.print("AT+CMGD=1,3\r");
  
  setupMPU();
//  calibrate();
  getCompensateVal();
  send_msg("Accident Detection System Started\n\nDrive safely and God bless.", "9503610262");
  
  //Serial.println("Started");
  delay(1000);
  //BT.println("Device Started");
  // BT.begin(9600);
  // delay(1000);
  lcd.clear();
}

void loop() {
  mainloop();
  if (!isAccident) {
    previousMillisResponse = millis();
    digitalWrite(buzz,LOW);
    delay(500);
    
  
    Gyro();

    Alarm();
    
    if(gpsAvailability){
      sprintf(str1, "    %3d KpH     ", gpsData[5]);
      sprintf(str2, "      %2d:%2d     ", gpsData[3], gpsData[4]);
    }else{
      sprintf(str1, "     No GPS     ");
      sprintf(str2, "  Check Wiring  ");
    }
    // btGetString();
  } else {
    digitalWrite(buzz,HIGH);
    accidentResponseCancel();
    accidentResponse();
    sprintf(str1,"AccidentDetected");
    sprintf(str2, "              %2d", 20-(millis()-previousMillisResponse)/1000);
  }

  #ifndef NGYRODEBUG
  recordGyroRegisters();
  printCoords(gForce);
  #endif
  
}

void getGPS() {
  S808.flush();
  unsigned int previousMillis = millis();
  S808.print("AT+CGPSOUT=32\r\n");
  while (millis() - previousMillis < 3000) {
    char temp = (char)S808.read();
    Serial.print(temp);
    if (gps.encode(temp)) {
      if(gps.date.isValid()&&gps.time.isValid()&&gps.location.isValid()){
        gpsData[0] = gps.date.month();
        gpsData[1] = gps.date.day();
        gpsData[2] = gps.date.year();
        gpsData[3] = gps.time.hour();
        gpsData[4] = gps.time.minute();
        gpsData[5] = int(gps.speed.kmph());
        Lat = gps.location.lat();
        Lon = gps.location.lng();
        gpsAvailability = true;
        if(!location_warn){
          send_msg("GPS Location Available", mainNumber);
          location_warn = true;
        }
        Serial.println("getGPS: Success");
        S808.print("AT+CGPSOUT=0\r\n");
        return;
      }
      
    }
  }
  S808.print("AT+CGPSOUT=0\r\n");
}

void accidentResponse() {
  if (millis() - previousMillisResponse > 20000) {
    if (Lat > 0.0 || Lon > 0.0){
      send_msg("Requesting assistance for vehicular accident at coordinates: https://www.google.com/maps/place/" + String(Lon,5) + "," + String(Lat,5) , mainNumber);
    }
    else {
      send_msg("Requesting assistance for vehicular accident, unfortunately the location was not determined during the accident.", mainNumber);
    }
    isAccident = false;
    digitalWrite(buzz, LOW);
    sprintf(str1,"sending sms rqst");
    sprintf(str2,"pls stand by    ");
    while(tilted){
      mainloop();
    }
    
  }
}

void accidentResponseCancel() {
  unsigned long previousMillisCancel = millis();
  
  while (digitalRead(btn) == 0) {
    if (millis() - previousMillisCancel > 5000) {
      isAccident = false;
      Serial.println("Request for assistance is cancelled.");
      sprintf(str1, "Assist Rqst     ");
      sprintf(str2, "Cancelled       ");
      lcdPrint();
      delay(3000);
      break;
    }
  }
}

#ifndef NBLUETOOTH
void btGetString() {
  String str = "";
  while (BT.available()) {
    str = BT.readString();
    #ifndef NBTDEBUG
    Serial.println(str);
    #endif

    if (str.charAt(0) == 'i') {
    BT.print("p");
    BT.print(alarm.flagEnabled);
    BT.print(";s");
    BT.print(alarm.Sensitivity);
    BT.print(";g");
    (gpsWarn) ? BT.print(1) : BT.print(0);
    } else if (str.charAt(0) == 'c') {
      calibrate();
    } else {
      btCom(str);
    }
  }
  
}

#endif


void printData(){
//  Serial.print("Gyro");
//  Serial.print(" X= ");
//  Serial.print(rot.X);
//  Serial.print("\t\tY= ");
//  Serial.print(rot.Y);
//  Serial.print("\t\tZ= ");
//  Serial.print(rot.Z);
  //Serial.print("\t\tAccel");
  Serial.print(" X=");
  Serial.print(gForce.X*90);
  Serial.print("\t\tY= ");
  Serial.print((1-gForce.Y)*90);
  Serial.print("\t\tZ= ");
  Serial.println((1-gForce.Z)*90);
}

void lcdPrint(){
  lcd.setCursor(0,0);
  lcd.print(str1);
  lcd.setCursor(0,1);
  lcd.print(str2);
}

void recv_msg(){
  
  if(S808.available()){
    textMessage = "";
    while (S808.available()) {
      textMessage += (char)S808.read();
      delay(5);
    }
    
    delay(100);
    textMessage.toLowerCase();
    Serial.println(textMessage);
    if(textMessage.indexOf("+cmgf: 0")>0 || textMessage.indexOf("+cmti")>0 ){
      S808.print("AT+ CMGF=1\r");
      analogWrite(buzz,255);
      delay(50);
      analogWrite(buzz,0);
      S808.print("AT+CNMI=2,2,0,0,0\r");
      delay(50);
      analogWrite(buzz,255);
      delay(50);
      analogWrite(buzz,0);
    }
    if (textMessage.indexOf("+cmt")>0) {
      analogWrite(buzz,255);
      //getPhoneNumber();
      S808.print("AT+CMGD=1,3\r"); //deletes recv read sms
      delay(250);
      analogWrite(buzz,0);
    }
    if(textMessage.indexOf("location")>=0 && (Lat>0||Lon>0)){
      textMessage = "https://www.google.com/maps/place/" + String(Lat, 5) + "," + String(Lon, 5);
      send_msg(textMessage, mainNumber);
    }else if(textMessage.indexOf("location")>0 && (Lat==0.0||Lon==0.0)){
      textMessage = "Location is not determined, Poor GPS signal";
      send_msg(textMessage, mainNumber);
    }else if(textMessage.indexOf("enable reply")>0){
      replyFlag=1;
      EEPROM.write(100,replyFlag);
      send_msg("Confirmation Reply Enabled", mainNumber);
    }else if(textMessage.indexOf("disable reply")>0){
      replyFlag=0;
      EEPROM.write(100,replyFlag);
    }else if(textMessage.indexOf("park mode on")>0){
      alarm.flagEnabled = true;
    }else if(textMessage.indexOf("park mode off")>0){
      alarm.flagEnabled = false;
    }else if(textMessage.indexOf("alarm 1")>0){
      alarm.Sensitivity = 1;
    }else if(textMessage.indexOf("alarm 2")>0){
      alarm.Sensitivity = 2;
    }else if(textMessage.indexOf("alarm 3")>0){
      alarm.Sensitivity = 3;
    }else if(textMessage.indexOf("power down") > 0){
      S808.print("AT+ CMGF=1\r");
      delay(50);
      S808.print("AT+CNMI=2,2,0,0,0\r");
      delay(50);
    }else if(textMessage.indexOf("calibrate")){
      calibrate();
    }

  }

}

void send_msg(String txt, String number){
  #ifdef ENABLE_SMS
  S808.print("AT+CMGF=1\r");
  delay(100);
  S808.println("AT+CMGS=\""+ number +"\"");
  delay(100);
  S808.print(txt);
  delay(100);
  S808.println((char)26);
  delay(1000);
  S808.println();
  delay(2000);
  #endif
}

void btCom(String x){
  if(x.indexOf("p1")>=0){
    alarm.flagEnabled = true;
  }else if(x.indexOf("p0")>=0){
    alarm.flagEnabled = false;
  }else if(x.indexOf("s1")>=0){
    alarm.Sensitivity = 1;
  }else if(x.indexOf("s2")>=0){
    alarm.Sensitivity = 2;
  }else if(x.indexOf("s3")>=0){
    alarm.Sensitivity = 3;
  }else if(x.indexOf("s0")>=0){
    alarm.Sensitivity = 0;
  }
}

void activateAlarm(Coords<float> avg) {
  int xRaw = 0;
  xRaw += abs((int)avg.X);
  xRaw += abs((int)avg.Y);
  xRaw += abs((int)avg.Z);
  xRaw /= 3;
  #ifndef NALARMDEBUG
  Serial.println(xRaw);
  #endif

  float x = xRaw;
  switch (alarm.Sensitivity) {  // todo: tweak the multipliers
    case 0:
      x = 0;
    case 1:
      x *= 0.75;
      break;
    case 2:
      break;
    case 3:
      x *= 1.5;
      break;
  }
  
  if (!alarm.flagOngoing)
  { 
    if(0<=x && x<=5){
    } else if (5<x && x<=15) {
      turnOnAlarm(3, 250, 750);
    } else if(15<x && x<=20){
      turnOnAlarm(5, 500, 500);
    }else if(20<x && x<=25){
      turnOnAlarm(7, 750, 250);
    }else{
      turnOnAlarm(10, 1000, 0);
    }
  }
}

void turnOnAlarm(int j, int x, int y){
  alarm.flagOngoing = true;
  alarm.Repetition = j;
  alarm.OnInterval = x;
  alarm.OffInterval = y;
  // int i = 0;
  // for(i;i<j;i++){
  // digitalWrite(buzz, HIGH);
  //   delay(x);
  //   digitalWrite(buzz, LOW);
  //   delay(y);
  // }
}

void Alarm() {
  if (alarm.flagOngoing) {
    if (alarm.Repetition > 0) {
      unsigned long currentMillisAlarm = millis();
      if (alarm.ptrCurInterval == nullptr) {    // check if it's the first time triggering the alarm
          previousMillisAlarm = currentMillisAlarm;
          analogWrite(buzz, 255);
          alarm.ptrCurInterval = &alarm.OnInterval;
          
          #ifndef NBEEPDEBUG
          Serial.println("*BEEEEEEEEEP*");
          #endif
      } 
      else if (currentMillisAlarm - previousMillisAlarm >= *alarm.ptrCurInterval) {
        previousMillisAlarm = currentMillisAlarm;
        
        if (alarm.ptrCurInterval == &alarm.OnInterval) {
          analogWrite(buzz, 0);
          alarm.ptrCurInterval = &alarm.OffInterval;
          alarm.Repetition--;

          #ifndef NBEEPDEBUG
          Serial.println("**");
          #endif
        }
        else {
          analogWrite(buzz, 255);
          alarm.ptrCurInterval = &alarm.OnInterval;

          #ifndef NBEEPDEBUG
          Serial.println("*BEEEEEEEEEP*");
          #endif
        }
      }
    } else {
      analogWrite(buzz, 0);
      alarm.flagOngoing = false;
      alarm.ptrCurInterval = nullptr;

      #ifndef NBEEPDEBUG
      Serial.println("**");
      #endif
    }
  }
}

void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);//waking up the module
  Wire.write(0x1B);//gyro parameters
  Wire.write(0x00000000); // range +/- 250 rpm
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000); //range +/- 8g 2048 LSB/g
  Wire.endTransmission();
}

void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  accel.X = Wire.read()<<8|Wire.read();
  accel.Y = Wire.read()<<8|Wire.read();
  accel.Z = Wire.read()<<8|Wire.read();
  processAccelData();
}

void processAccelData(){
  gForce.X = fabs(accel.X / 16384.0);
  gForce.Y = fabs(accel.Y / 16384.0);
  gForce.Z = fabs(accel.Z / 16384.0);
  compensate();
}

void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  gyro.X = Wire.read()<<8|Wire.read();
  gyro.Y = Wire.read()<<8|Wire.read();
  gyro.Z = Wire.read()<<8|Wire.read();
  processGyroData();
}

float fabs(float val) {
  return (val < 0) ? val * -1 : val;
}

void processGyroData(){
  rot.X = gyro.X / 131.0;
  rot.Y = gyro.Y / 131.0;
  rot.Z = gyro.Z / 131.0;
}

void calibrate(){
  recordAccelRegisters();
  gForce.X = fabs(accel.X / 16384.0);
  gForce.Y = fabs(accel.Y / 16384.0);
  gForce.Z = fabs(accel.Z / 16384.0);
  int x = gForce.X*100.0, y = gForce.Y*100.0, z = gForce.Z*100.0;
  EEPROM.update(500, 143); //just a flag
  delay(100);
  EEPROM.update(501, x);
  delay(100);
  EEPROM.update(502, y);
  delay(100);
  EEPROM.update(503, z);
  delay(100);
}

void getCompensateVal(){
  if(EEPROM.read(500)==143){
    compensateVal.X = EEPROM.read(501)/100.0;
    compensateVal.Y = EEPROM.read(502)/100.0;
    compensateVal.Z = EEPROM.read(503)/100.0;
  }else{
    calibrate();
  }
}

void compensate(){
  gForce.X = fabs(gForce.X - compensateVal.X);
  gForce.Y = fabs(gForce.Y - compensateVal.Y);
  gForce.Z = fabs(gForce.Z - compensateVal.Z);
}

void Gyro() {
  unsigned long currentMillisGyro = millis();
  if (currentMillisGyro - previousMillisGyro >= GYROINTERVAL) {
    previousMillisGyro = currentMillisGyro;
    
    recordGyroRegisters();
    gyroBuffer.insert(rot);

    if (gyroBuffer.getCount() == GYROBUFFERSIZE) {
      Coords<float> avg {};
      for (int i = 0; i<GYROBUFFERSIZE; i++)
        avg += gyroBuffer[i];
      avg = avg/GYROBUFFERSIZE;

      if (alarm.flagEnabled) {
        activateAlarm(avg); // rename this func name?
      }
      #ifndef NALARMDEBUG
      printCoords(avg);
      #endif
    }
  }
}


void Tilt() {
  recordAccelRegisters();
  
  if (gForce.X > 0.8 || gForce.Y > 0.8 || gForce.Z > 0.8) {
    if(!tilted && !isAccident && 30<(millis()-previousAccident)/1000){
      isAccident = true;
      previousAccident = millis();
    }
    tilted = true;
    
  }else{
    tilted = false;
  }
  
  #ifndef NACCELDEBUG
  printCoords(gForce);
  #endif
}

template <typename T>
void printCoords(Coords<T> coords) {
  Serial.print(coords.X);
  Serial.print('\t');
  Serial.print(coords.Y);
  Serial.print('\t');
  Serial.println(coords.Z);
}

void mainloop(){
  if(millis()>5000&&gps.charsProcessed()<10){
    gpsAvailability = false;
    Serial.println("no gps");
  }
  getGPS();
  //delay(100);
  #ifndef NGPSDEBUG
  Serial.println(String(Lon, 5) + "," + String(Lat, 5));
  #endif
  recv_msg();
  Tilt();
  lcdPrint();
}

// +CMT: "+639503610262","","19/08/23,21:13:18+32"
// Tndnxnc
// ===============
// UNDER-VOLTAGE WARNNING
