#include <math.h>
#include "SoftPWM.h"
#include "tempTable.h"
#include <SoftwareSerial.h>
#include <Wire.h>  // Comes with Arduino IDE

// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include "LiquidCrystal_I2C.h"

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

bool setpointsChanged = 1;

int extruderHeatingMode = 0;
#define extruderHeat_BangBang 0
#define extruderHeat_PID 1


#define eKp 0.15
#define eKi 0.02
#define eKd 0.0005

#define TXPin 3
#define RXPin 2
#define EHeatPin 12
#define BHeatPin1 4
#define BHeatPin2 5
#define BHeatPin3 7
#define BHeatPin4 9

#define ESensePin A0
#define BSensePin1 A3
#define BSensePin2 A7
#define BSensePin3 A6
#define BSensePin4 A1

#define controlPeriod 200 //ms
#define bedControlPeriod 500 //ms
#define reportPeriod 400 //ms
#define lcdPeriod 2000   //ms

#define IDLING  0
#define HEATING 1
#define READY   2
#define COOLING 3

//AKTIVOI LCD VIIMEISEN RIVIN DEBUG TULOSTEEN
#define DEBUG 0

void reportToSerial(void);
void readTemperatures(void);

SoftwareSerial kFlopSerial(RXPin, TXPin); // RX, TX

long lastControlTime = 0;
long lastBedControlTime = 0;
long lastReportTime = 0;
long lastLCDUpdateTime = 0;

int eSetPoint = 0;
int bSetPoint = 0;
int eEffort = 0;
int bHeatIndex = 0;

int processStatus = IDLING;
float eErrSum = 0.0;
float lastEErr = 0.0;

float eTemp = 0;
float bTemp1 = 0;
float bTemp2 = 0;
float bTemp3 = 0;
float bTemp4 = 0;

void setup() {
  pinMode(RXPin, INPUT);
  pinMode(TXPin, OUTPUT);
  pinMode(ESensePin, INPUT);
  pinMode(BSensePin1, INPUT);
  pinMode(BSensePin2, INPUT);
  pinMode(BSensePin3, INPUT);
  pinMode(BSensePin4, INPUT);
  pinMode(EHeatPin, OUTPUT);
  pinMode(BHeatPin1, OUTPUT);
  pinMode(BHeatPin2, OUTPUT);
  pinMode(BHeatPin3, OUTPUT);
  pinMode(BHeatPin4, OUTPUT);
  //digitalWrite(EHeatPin, LOW);
  digitalWrite(BHeatPin1, LOW);
  digitalWrite(BHeatPin2, LOW);
  digitalWrite(BHeatPin3, LOW);
  digitalWrite(BHeatPin4, LOW);

  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.clear();
  lcd.setCursor(0,0); //Start at line 0
  lcd.print("Castor Mega Printer");
  lcd.setCursor(0,1);
  lcd.print("Booting up!");
  delay(7000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SP: E:   " + (String)eSetPoint);
  lcd.setCursor(13,0);
  lcd.print(" B:   " + (String)bSetPoint);
  lcd.setCursor(0,1);

  int eTempInt = (int)eTemp+0.5;
  lcd.print("PV: E:");
  if(eTempInt < 100) {
    lcd.print(" ");
  }
  if(eTempInt < 10) {
    lcd.print(" ");
  }
  lcd.print((String)eTempInt);
  lcd.setCursor(0,2);
  lcd.print("B: ");
  int temp =(int)bTemp1+0.5;
  if(temp<100) lcd.print(" ");
  if(temp<10) lcd.print(" ");
  lcd.print((String)temp + " ");
  
  temp =(int)bTemp2+0.5;
  if(temp<100) lcd.print(" ");
  if(temp<10) lcd.print(" ");
  lcd.print((String)temp + " ");

  temp =(int)bTemp3+0.5;
  if(temp<100) lcd.print(" ");
  if(temp<10) lcd.print(" ");
  lcd.print((String)temp + " ");

  temp =(int)bTemp4+0.5;
  if(temp<100) lcd.print(" ");
  if(temp<10) lcd.print(" ");
  lcd.print((String)temp);

  lcd.setCursor(4,3);
  lcd.print("Idle");

  SoftPWMBegin();
  SoftPWMSet(EHeatPin, 0);

  Serial.begin(115200);
  kFlopSerial.begin(4800);
}



void loop() {
  if (kFlopSerial.available()) {
    readkFlopCommand();
  }

  if (millis() > (controlPeriod + lastControlTime)) {
    runExtruderPID(millis() - lastControlTime);
    lastControlTime = millis();
  }

  if (millis() > (bedControlPeriod + lastBedControlTime)) {
    runBedControl();
    lastBedControlTime = millis();
  }

  if (millis() > (reportPeriod + lastReportTime)) {
    reportToSerial();
    lastReportTime = millis();
  }

  readTemperatures();

  if (millis() > (lcdPeriod + lastLCDUpdateTime)) {
    updateLCD();
    lastLCDUpdateTime = millis();
  }
}



void updateLCD() {
  if(setpointsChanged) {
    lcd.setCursor(7,0);
    if(eSetPoint<100){
      lcd.print(" ");
    }
    if(eSetPoint<10){
      lcd.print(" ");
    }
    lcd.print((String)eSetPoint);
    lcd.setCursor(17,0);
    lcd.print((String)bSetPoint);
    if(bSetPoint<100){
      lcd.print(" ");
    }
    if(bSetPoint<10){
      lcd.print(" ");
    }
    setpointsChanged=0;
  }
  long starttest=millis();
  lcd.setCursor(7,1);
  int eTempInt = (int)eTemp+0.5;
  if(eTempInt<100) {
    lcd.print(" ");
  }
  if(eTempInt<10) {
    lcd.print(" ");
  }
  lcd.print((String)eTempInt);
  
  lcd.setCursor(3,2);
  int temp1 =(int)bTemp1+0.5;
  if(temp1<100) lcd.print(" ");
  if(temp1<10) lcd.print(" ");
  lcd.print((String)temp1 + " ");
  
  int temp2 =(int)bTemp2+0.5;
  if(temp2<100) lcd.print(" ");
  if(temp2<10) lcd.print(" ");
  lcd.print((String)temp2 + " ");

  int temp3 =(int)bTemp3+0.5;
  if(temp3<100) lcd.print(" ");
  if(temp3<10) lcd.print(" ");
  lcd.print((String)temp3 + " ");


  int temp4 =(int)bTemp4+0.5;
  if(temp4<100) lcd.print(" ");
  if(temp4<10) lcd.print(" ");
  lcd.print((String)temp4);
  
  long endtest=millis();

  // determine general temp status
  // and print a new status only if needed.
  
  int status = IDLING;
  if ((eSetPoint < 1) && (bSetPoint < 1)) {
    // setpointit tarpeeksi pieniä  -> ollaan varmasti idlaamassa
    status=IDLING;
  }
  else if ((abs(eSetPoint-eTemp) > 15) || abs((bSetPoint-temp1) > 15) || abs((bSetPoint-temp2) > 15) || abs((bSetPoint-temp3) > 15) || abs((bSetPoint-temp4) > 15)) {
    // setpointit ei ole liian matalat, ja mikä vaan lämpö on ainakin 5 astetta setpointtia pienempi
    // -> lämmitetään vielä
    status=HEATING;
  }
  else {
      status=READY;
  }
  
  // processStatus is global variable
  if (status != processStatus) {
    
    // need to update status screen and processStatus value
    processStatus = status;
    lcd.setCursor(4,3);
    switch (status) {
      case IDLING:
        lcd.print("Idle           ");
        break;
      case HEATING:
        lcd.print("Heating        ");
        break;
      case READY:
        lcd.print("Process ready! ");
        break;
    }
  }

  if(DEBUG) {
    //debug tulostukset
    //viimeisen rivin alkuun
    lcd.setCursor(0,3);
    //lcd.print((String)(endtest-starttest));
    lcd.print("PI:");
    lcd.print((String)eErrSum);
    lcd.print("MD:");
    lcd.print((String)extruderHeatingMode);
  }

}



void runExtruderPID(double time) {
  float eErr = (int(eSetPoint*100) - (int(eTemp*100)));
  eErr /= 100.0;
  
  if (eSetPoint > 1.0) {  
    if ((eErr < 35.0) || (eTemp > 200.0)) {
        extruderHeatingMode = extruderHeat_PID;
     } else extruderHeatingMode = extruderHeat_BangBang;
    
    if(extruderHeatingMode == extruderHeat_BangBang) {
        //run bangbang-mode
        SoftPWMSet(EHeatPin, 100);
    } else {
      eEffort = 0;
      
      eErrSum += (eErr * time)/8;
      eErrSum = constrain(eErrSum, -800,450);
      
      double dErr = (eErr - lastEErr) / time;
      lastEErr = eErr;
    
      // PID tulos
      eEffort = constrain(eSetPoint*eKp + eErrSum*eKi + dErr*eKd, 0, 80);  
      
      //Write E fet current, range 0-250 in eEffort
      SoftPWMSet(EHeatPin, eEffort);
    }
  } else SoftPWMSet(EHeatPin, 0);
}



void runBedControl(){
  // Write all low
  digitalWrite(BHeatPin1, LOW);
  digitalWrite(BHeatPin2, LOW);
  digitalWrite(BHeatPin3, LOW);
  digitalWrite(BHeatPin4, LOW);

  //bHeatIndex tells which segment has permission to use power
  int segmentsHeating = 0;
  int orgHeatIndex = bHeatIndex;

  if (bHeatIndex == 0) {
    if ((int)bTemp1 < bSetPoint) {
      digitalWrite(BHeatPin1, HIGH);
      segmentsHeating++;
    }
    if ((int)bTemp3 < bSetPoint) {
      digitalWrite(BHeatPin3, HIGH);
      segmentsHeating++;
    }
    if (segmentsHeating < 2) bHeatIndex = 1;
  }

  if (bHeatIndex == 1) {
    if ((int)bTemp2 < bSetPoint) {
      digitalWrite(BHeatPin2, HIGH);
      segmentsHeating++;
    }
    if (((int)bTemp4 < bSetPoint) && (segmentsHeating < 2)) {
      digitalWrite(BHeatPin4, HIGH);
      segmentsHeating++;
    }
  }

  if ((orgHeatIndex == 1) && (segmentsHeating < 2)) {
    if ((int)bTemp1 < bSetPoint) {
      digitalWrite(BHeatPin1, HIGH);
      segmentsHeating++;
    }
    if (((int)bTemp3 < bSetPoint) && (segmentsHeating < 2)) {
      digitalWrite(BHeatPin3, HIGH);
      segmentsHeating++;
    }
  }

  // Advance power use turn
  bHeatIndex++;
  if (bHeatIndex >= 2) bHeatIndex = 0;
}

void readkFlopCommand() {
  //Serial.println((char)kFlopSerial.read());
  char c = ' ';
  int i = 0;
  char input[5];
  for (i=0; i<5; i++) {
    input[i] = 0;
  }
  i=0;
  
  while (kFlopSerial.available()) {
    c = kFlopSerial.read();
    Serial.print("Tuli: ");
    Serial.println((char)c);
    if( c != ':') {
      input[i] = c;
      i++;
    } else {
    break;
    }
    delay(50);
  }

  char numero[4];
  numero[3]=0;
  numero[2]=input[1];
  numero[1]=input[2];
  numero[0]=input[3];
  if(input[0]=='e' ) {
    Serial.println("Setting E setpoint!");
    eSetPoint = atoi(numero);
    setpointsChanged=1;
  } else if(input[0] == 'b') {
    Serial.println("Setting B setpoint!");
    bSetPoint = atoi(numero);
    setpointsChanged=1;
  } else if ((input[0] == 'c')) {
    replyStatus();
  } else {
    Serial.println("Setting unknown setpoint WTF?!");
  }
}

void replyStatus() {
  switch (processStatus) {
     case IDLING:
        kFlopSerial.print("idle");
        break;
      case HEATING:
        kFlopSerial.print("heating");
        break;
      case READY:
        kFlopSerial.print("ready");
        break;
  }
}

void reportToSerial() {
  Serial.println("Et\tBt 1\tBt 2\tBt 3\tBt 4");
  String temps = (String)eTemp + "\t" + (String)bTemp1 + "\t" + (String)bTemp2 + "\t" + (String)bTemp3 + "\t" + (String)bTemp4 + "\t";
  Serial.println(temps);
  Serial.print("E setpoint: " + (String)eSetPoint + (String)"\n");
  Serial.print("B setpoint: " + (String)bSetPoint + (String)"\n");
  Serial.print("E Power ");
  Serial.println(eEffort);
}

void readTemperatures() {
  eTemp  = (2.0*(convertThermistorTable(analogRead(ESensePin))) + 98.0*eTemp)/100.0;
  bTemp1 = (2.0*(convertThermistorTable(analogRead(BSensePin1))) + 98.0*bTemp1)/100.0;
  bTemp2 = (2.0*(convertThermistorTable(analogRead(BSensePin2))) + 98.0*bTemp2)/100.0;
  bTemp3 = (2.0*(convertThermistorTable(analogRead(BSensePin3))) + 98.0*bTemp3)/100.0;
  bTemp4 = (2.0*(convertThermistorTable(analogRead(BSensePin4))) + 98.0*bTemp4)/100.0;
}

float convertThermistorTable(int rawADC) {
  int i = 0;
  int solution = false;
  // Seek our high value in table
  for (i; i < 60;i++) {
    if (rawADC < (int)(temptable[i][0])) {solution = true; break;}
  }
  if (solution) {
    int highADC = (int)(temptable[i][0]);
    int lowTemp = (int)(temptable[i][1]);
    int lowADC = (int)(temptable[i-1][0]);
    int highTemp = (int)(temptable[i-1][1]);
    //Serial.println("Begin");
    //Serial.println(lowADC);
    //Serial.println(rawADC);
    //Serial.println(highADC);
    int residualADC = rawADC - lowADC;
    if (residualADC != 0) {
      float adcPropo = 1.0 - (float)residualADC / ((float)(highADC - lowADC));
      float residualTemp = float((highTemp - lowTemp)) * adcPropo;
      return residualTemp + lowTemp;
    } else return (float)lowTemp;
  } else return 0.0;
}

