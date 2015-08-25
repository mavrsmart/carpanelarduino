#include <EEPROM.h>
//#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

#define rxPin 5
#define txPin 4
#define pwmklap 6
#define speedpin 3
#define led 13

 SoftwareSerial mySerial(5, 4); // RX, TX
 OneWire oneWire(ONE_WIRE_BUS);
 DallasTemperature sensors(&oneWire);


int si=0;
char buf[100];
char command_in[10];
char data_in[100];

unsigned char pwmklapval=0;
volatile int state = LOW;
float temp;
/*
$GPGGA,151611.103,8960.0000,N,00000.0000,E,0,0,,137.0,M,13.0,M,,*42
$GPGLL,8960.0000,N,00000.0000,E,151611.103,V,N*47
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,1,1,02,15,,,30,18,,,33*75
$GPRMC,151611.103,V,8960.0000,N,00000.0000,E,0.00,0.00,160815,,,N*7B
$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32
$GPZDA,151611.103,16,08,2015,,*5E

 */

unsigned char param_klap_temp=20;//желаемая температура
unsigned char param_max_klap_temp=10;//до этой температуры жарить


float temp_param;
float temp_max_param;
float temp_res;


char st = '0';
char st1 = '0';
char b=0;
char p=0;
String time;
String date;
String lat;
String lat_s;
String lon;
String lon_s;
String qual;
String sats;
String speed;
String angle;
String alt;
#define maxbuf 10
float speedsred[maxbuf];
String speeddat;

boolean renew = false;
boolean renew1 = false;
unsigned int tim;


ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = 0;   // preload timer
  tim=0;
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}

void speedtick(){
  tim=TCNT1;
 // sprintf(buf,"%d",tim);
 // speeddat=buf;
  TCNT1  =0;
  digitalWrite(led, 1);
}


void setup() {
//timer1
noInterrupts(); 
TCCR1A = 0;
TCCR1B = 0;
TCNT1 = 0;

//OCR1A = 31250; // compare match register 16MHz/256/2Hz
    TIMSK1 = (1 << TOIE1);
    // Set CS10 bit so timer runs at clock speed:
    TCCR1B |= (0 << CS10);
    TCCR1B |= (0 << CS11);
    TCCR1B |= (1 << CS12);
TCNT1 = 0;
    pinMode(speedpin, INPUT);
  pinMode(led, OUTPUT);
  attachInterrupt(1, speedtick, CHANGE);
  interrupts();
  // put your setup code here, to run once:
  Serial.begin(57600);

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  mySerial.begin(9600);
  mySerial.write("$PMTK251,38400*27\r\n");
  mySerial.write("$PMTK101*32\r\n");
  //mySerial.write("$PMTK251,38400*27");
  //mySerial.write("$PMTK220,250*29\r\n");
  mySerial.write("$PMTK220,1000*1F\r\n");
  mySerial.begin(38400);
  mySerial.write("$PMTK220,1000*1F\r\n");
  mySerial.write("$PMTK103*32\r\n");
for(int i=0;i<maxbuf;i++){
  speedsred[i]=0;
}
sensors.begin();
sensors.requestTemperatures();

param_klap_temp=EEPROM.read(0x00);
if(param_klap_temp==255){
  EEPROM.write(0x00,20);
  param_klap_temp=EEPROM.read(0x00);
  }

param_max_klap_temp=EEPROM.read(0x01);
if(param_max_klap_temp==255){
  EEPROM.write(0x01,10);
  param_max_klap_temp=EEPROM.read(0x01);
  }


}


String readVal() {
boolean rot = true;
String buffer;
while(rot) {
  if (mySerial.available()>0){
char b = mySerial.read();
//Serial.write(b);
if ( (('0' <= (char)b) && ((char)b <= '9')) || ((char)b == '.') || (('A' <= (char)b) && ((char)b <= 'Z')) ) { // считываем значения, содержащие цифры, буквы и точку
buffer += (char)b; 
}
if ( (char)b == ',' ) rot = false; // заканчиваем читать, когда встречаем запятую
//delay(1); 
  }
}
return buffer;
}

void loop() {
si=0; 
//если есть данные - читаем
  if(Serial.available()){
     delay(50);
      
     //загоняем прочитанное в буфер
     while((Serial.available()) && (si< 99)) {
        buf[si++] = Serial.read();
     }
     //закрываем массив
     buf[si++]='\0';
    //разбераем его на части отделенные запятой
    sscanf(buf, "%[^','],%s", &command_in, &data_in);
  
    Serial.print("Input command: ");
    Serial.print(buf);
    Serial.print("\r\n");

//Обрабатываем комманды
   if ((String)command_in == "stk"){//комманда установки температуры клапана
     param_klap_temp=atoi(data_in);
     EEPROM.write(0x00,param_klap_temp);
   }
   if ((String)command_in == "stmk"){//комманда установки температуры клапана жаровни
     param_max_klap_temp=atoi(data_in);
     EEPROM.write(0x01,param_max_klap_temp);
   }
   }
//конец обработки комманды

  
  /*if (mySerial.available())
    Serial.write(mySerial.read());
delay(1);    */
if (mySerial.available()>0){  
b= mySerial.read();
//Serial.write(b);
//Serial.write(st);


switch ( st ) { // перебираем состояния
case '0': if('$' == b) st = '1';
break;
case '1': if('G' == b) st = '2'; else st = '0';
break;
case '2': if('P' == b) st = '3'; else st = '0';
break;
case '3': if('G' == b) st = '4'; else st = '0';
break;
case '4': if('G' == b) st = '5'; else st = '0';
break;
case '5': if('A' == b) st = '6'; else st = '0';
break;
case '6': if(',' == b) { // распознали $GPGGA
time = readVal(); 
lat = readVal();
lat.replace('.',',');
lat_s = readVal();
lon = readVal();
lon.replace('.',',');
lon_s = readVal();
qual = readVal();
sats = readVal();
readVal(); // skip empty
alt = readVal();

renew = true;
st = '0';
} else st = '0';
break;

}


switch ( st1 ) { // перебираем состояния
case '0': if('$' == b) st1 = '1';
break;
case '1': if('G' == b) st1 = '2'; else st1 = '0';
break;
case '2': if('P' == b) st1 = '3'; else st1 = '0';
break;
case '3': if('R' == b) st1 = '4'; else st1 = '0';
break;
case '4': if('M' == b) st1 = '5'; else st1 = '0';
break;
case '5': if('C' == b) st1 = '6'; else st1 = '0';
break;
case '6': if(',' == b) { // распознали $GPGGA

//$GPRMC,151611.103,V,8960.0000,N,00000.0000,E,0.00,0.00,160815,,,N*7B

//time = 
readVal(); 
readVal();
//lat = 
readVal();
//lat_s = 
readVal();
//lon = 
readVal();
//lon_s = 
readVal();
speed = readVal();
angle = readVal();
date = readVal(); // skip empty
//alt = readVal();

renew1 = true;
st1 = '0';
} else st1 = '0';
break;

}


}

if (renew&&renew1) {
  if(b=='\n'){

  
//Serial.print("Start\r\n");
Serial.print("Lat: " + lat_s + lat +"\r\n");
Serial.print("Lon: " + lon_s + lon + "\r\n");
Serial.print("Tim: " + time + "\r\n");
Serial.print("Dat: " + date + "\r\n");
Serial.print("Qty: " + qual + "\r\n");
Serial.print("Sat: " + sats+"\r\n");
Serial.print("SpdGPS: " + speed+"\r\n");
speeddat=String((1/((tim*0.000016)*6))*(3600/1000));
Serial.print("SpdDat: " + speeddat+"\r\n");
Serial.print("Temp: ");
temp=sensors.getTempCByIndex(0);
Serial.println(temp);  
Serial.print("Param klap temp: "+String(param_klap_temp,DEC)+"\r\n");
Serial.print("Param Max klap temp: "+String(param_max_klap_temp,DEC)+"\r\n");
Serial.print("PWM klap temp: "+String(pwmklapval,DEC)+"\r\n");

//Serial.print("--------------------\r\n");
renew = false;
renew1 = false;

//////////шим клапана
temp_param=(float)param_klap_temp;
temp_max_param=(float)param_max_klap_temp;
temp_res=0.0;

if(temp<=temp_max_param){
  pwmklapval=255;
}
if(temp>temp_max_param){
//   temp_res=(temp_param-temp)/((temp_param-temp)/255);
   temp_res=(temp_param-temp_max_param)/255.0;
   temp_res=(temp_param-temp)/temp_res;
   pwmklapval=(unsigned char)temp_res;
   if(temp>=temp_param){
      pwmklapval=0;
   }
}

analogWrite(pwmklap, 255-pwmklapval); 

///////////не шим клапана

sensors.requestTemperatures();

}
}




//delay(1);
digitalWrite(led, 0);

}
