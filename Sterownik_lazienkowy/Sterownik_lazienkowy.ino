#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DS1307.h>
#include <AT24Cxx.h>
#include <Fotorezystor.h>
#include <Przekaznik.h>


// inicjalizacja LCD
LiquidCrystal_I2C lcd(0x20,4,5,6,0,1,2,3,7,NEGATIVE);  // set the LCD address to 0x20 for a 16 chars and 2 line display
byte customChar[8] = {
  0b01000,
  0b10100,
  0b01000,
  0b00011,
  0b00100,
  0b00100,
  0b00100,
  0b00011
};

class Wentylator{
  protected:
    boolean Pracuje;//info czy wentylator pracuje
    byte Histereza;//histereza
    byte pinPrzekaznika;//pin przekaznika na ktory załączy wntylator

  public:
    void Begin(byte pin,byte hist){
      this->pinPrzekaznika=pin;
      this->Pracuje=false;
      this->Histereza=hist;
    }
    void AutomatWentylator(byte WilgWzor,int WilgPomiar){
      //Serial.print("wentylator");Serial.println(Pracuje);
      //Serial.print("Wzor");Serial.println(WilgWzor); 
      //Serial.print("Pomiar");Serial.println(WilgPomiar);
    if((WilgWzor<WilgPomiar) || (Pracuje)){
    Serial.print("WilgotnoscWzor");Serial.print(WilgWzor);
    if(WilgWzor-Histereza<WilgPomiar){
      Serial.print("WentylatorZalacz");Serial.print(Pracuje);
      digitalWrite(pinPrzekaznika,LOW);
      this->Pracuje=true;  
    }
      
  }
  else{
    digitalWrite(pinPrzekaznika,HIGH);
    this->Pracuje=false;
  }
  
}
    byte AktuHist(byte tempHist){
      this->Histereza=tempHist;
      return tempHist;  
    }
    boolean OnWentylator(){
      if(Pracuje){
        digitalWrite(this->pinPrzekaznika,HIGH);
        this->Pracuje=false;
        return false;
        //break;
      }
      else{
        digitalWrite(this->pinPrzekaznika,LOW);
        this->Pracuje=true;
        return true;
        //break;
      }
    }
};


Fotorezystor F;
// zegar i EEPROM układ Tiny RTC DS1307
DS1307 clock;
RTCDateTime dt;

AT24Cxx eep(0x50, 32);

Wentylator W;


 // Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 11
#define pinDHT 6//pin DHT22
#define pirPin  3 //pin czujnika ruchu PIR
#define pinRelayLight 5//pin przekaznika swiatła
#define pinRelayWent 7//pin przekaznika wentylatora
#define pinButtonNext 4//pin przycisku NEXT
#define pinButtonOK 8//pin przycisku OK
#define HisterezaWentylatora 10//histereza wyłaczenia wentylatora w %
#define pinLED 13// dioda LED

char* text;
char* Menu[]={"temperatura     ","Histereza wentyl","wentylator      ","czujnik ruchu   ","ust.daty i czasu",/*"Stan baterii RTC",*/"Czujnik zmierzchu ","koniec          "};
// inicjalizacja DHT 22 czujnik wilgoci
DHT dht;

Przekaznik P;


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempDeviceAddress;
DeviceAddress tempDeviceAddressEEPROM[]={0};
byte numberOfDevices; // Number of temperature devices found

byte HisterezaTemp=2;
// connectors on bottom
/*
--------------------
|                  |
|                  |
|                  |
|       o o o      |
|      5V o GND    |
--------------------
*/
/////////////////////////////
// voltage 4,5-20V
//

//VARS
//byte DaneDS18B20 [][8] ={{0},{0}};
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
byte calibrationTime = 30;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 10;  

boolean lockLow = true;
boolean takeLowTime;  

// Timer
class MyTimer{
  protected:
    
    unsigned long CzasUruchomienia;//czas uruchomienia 
    
    boolean Stan;// stan wykorzystania Timera czy 
  public:
    void Begin(){
      this->CzasUruchomienia=millis();
      this->Stan=1;
    }
    boolean Sprawdzaj(unsigned long Czekam){
      if(Stan){
        if(millis()-CzasUruchomienia>Czekam){
          this->CzasUruchomienia=millis();
          this->Stan=0;
          
                  
        }
      }  
      else{
         this->Stan=1;
      
      }
      return this->Stan;
    }
};
unsigned long TimeLastStart=0;
// dane z EEPROM dotyczące wilgotnosci
byte Wilgotnosc;
MyTimer Czas;
byte PoziomZmierzchu;
//-------------------------------------------------------------
void setup(void)
{
  F.Begin(A1);
  Czas.Begin();
  Serial.begin(9600);
  text="Inicjalizacja wyświetlacxza LCD...";
   Serial.println(text);
   lcd.begin(16,2);                      // initialize the lcd 
   lcd.backlight();
   lcd.createChar(0, customChar);
  
  P.Init(pinRelayLight,0);
   text="Initialize DS1307";
  Serial.println(text);
  clock.begin();

//wyswietlenie czasu 
  wyswietlenieCzasu();

//---------------------
   pinMode (pinButtonNext,INPUT_PULLUP);// pin 4 dla przycisku podłaczonego do GND
   pinMode (pinButtonOK,INPUT_PULLUP);// pin 8 dla przycisku podłaczonego do GND
   //pinMode(pinRelayLight, OUTPUT);//pin przekaźnika automat z PIR
   pinMode(pinRelayWent,OUTPUT);//pin przekaźnika przycisku  
   pinMode(pirPin, INPUT);//sekcja czujnika PIR
   pinMode(pinLED,OUTPUT);//dioda LED sygnalizacyjna
   digitalWrite(pinRelayLight,HIGH);//ustalenie stanu poczatkowego
   digitalWrite(pinRelayWent, HIGH);
   digitalWrite(pinLED,LOW);
   dht.setup(pinDHT); // data pin 6
   
   Serial.println(dht.getModel());
   
   
   wyswietlenieTytulu(); 
    
  //pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);
  lcd.clear();
  lcd.home();
  //give the sensor some time to calibrate
  //Serial.println("calibrating sensor ");
  text="kalibracja PIR";
  lcd.print(text);
  
    //kalibracja czujnika PIR
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      lcd.setCursor(0,1);
      lcd.print(i+1);
      lcd.print(" z ");
      lcd.print(calibrationTime);
      delay(1000);
      
      }
    //Serial.println(" done");
    //Serial.println("SENSOR ACTIVE");
    //Serial.print("czas czuwania PIR(milisek):");
    
    pause=eep.read(11)*60000;
    Wilgotnosc=eep.read(10);
    HisterezaTemp=eep.read(9);
    PoziomZmierzchu=eep.read(8);
    
    
    W.Begin(pinRelayWent,HisterezaTemp);
    //Serial.println(pause);
    //delay(50);
  
  
  
  //Serial.println("Dallas Temperature IC Control Library Demo");
  // locate devices on the bus
  //Serial.print("Locating devices...");

  sensors.begin();
  text="Found ";
  Serial.print(text);
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  Serial.print(numberOfDevices, DEC);
  text=" devices.";
  Serial.println(text);

  // report parasite power requirements
  text="Parasite power is: ";
  Serial.print(text); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  // Loop through each device, print out address
  for(byte i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      text="Odczytano adres termometru z pamięci EEPROM";
      Serial.println(text);
      for(byte temp=0;temp<8;temp++)
      {
        //odczytanie z EEPROM adresów DS18B20
        tempDeviceAddressEEPROM [i][temp]=eep.read((8*i)+(2000+temp)+i);
        Serial.print(tempDeviceAddressEEPROM [i][temp],HEX);
      }
      if (strcmp(tempDeviceAddress,tempDeviceAddressEEPROM[i]) == 0)
      {
        text="termometr jest juz zapisany w EEPROM";
        Serial.println(text);   
      }
      else
      {
        text="Dodawanie nowego termometru";
        Serial.println(text);
        for(byte temp=0;temp<8;temp++)
        {
          eep.update((8*i)+(2000+temp)+i,tempDeviceAddress[temp]);
          Serial.println(tempDeviceAddress[temp],HEX);
        }
        Serial.println("Dodano nowy termometr do pamieci EEPROM");
      }
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress,i);
      Serial.println();    
      Serial.print("Setting resolution to ");
      Serial.println(TEMPERATURE_PRECISION, DEC);    
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      Serial.print("Resolution actually set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
      Serial.println();
    }
    else{
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  lcd.clear();
  lcd.noBacklight();
}



void loop(void)
{ 
  
  MenuSystem(pinButtonNext,pinButtonOK,pinRelayLight,pinRelayWent);
  odczytDHT();
  OdczytTemperaturDS18B20();
  
   if(Czas.Sprawdzaj(60000)){
     // if (clock.isReady()){
        dt = clock.getDateTime();
        lcd.setCursor(0,1);
        //text=dt.hour+":"+dt.minute;
        //Serial.print(text);
        //lcd.print(text);
        lcd.print(dt.year);   lcd.print("-");
        print2digits(dt.month);  lcd.print("-");
        print2digits(dt.day);    lcd.print(" ");
        print2digits(dt.hour);   lcd.print(":");
        print2digits(dt.minute);
     // }
   }
 
 loopPIR();
}

//urzywane do wyswietlania czasu zmienia wyswietlane liczby na 2cyfrowe
void print2digits(byte number) {
  if (number >= 0 && number < 10) {
    lcd.print('0');
  }
  lcd.print(number);
}

//odczyt temperatur z Ds18B20
void OdczytTemperaturDS18B20(){
//text="Requesting temperatures...";
 // Serial.print(text);
  sensors.requestTemperatures(); // Send the command to get temperatures
  
 // Serial.println("DONE");
  
  
  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.println(i,DEC);
    
      // It responds almost immediately. Let's print out the data
      //float temperatura=GetToPrintTemperature(tempDeviceAddress); // Use a simple function to print out the data
    
      lcd.setCursor(10,i);
      lcd.print(GetToPrintTemperature(tempDeviceAddress));
      text=" ";
      lcd.print((char)0);lcd.print(text);
    
    
    } 
 
  //else ghost device! Check your power requirements and cabling
  
  }
}

// function to print the temperature for a device
int GetToPrintTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  int tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.println(tempC);
  //lcd.setCursor(11,0); 
  //lcd.print(tempC);
  //Serial.print(" Temp F: ");
  //lcd.setCursor(10,1); 
  //lcd.print(tempC);
  //Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  return tempC;
}

 //wyswietlenie tytuł
void wyswietlenieTytulu()
{
  lcd.home();
  
  lcd.print("Sterownik lazienkowy");
  lcd.setCursor(0, 1); 
  lcd.print("v1.1 29.02.2016");
  delay(1000);
  lcd.clear();
}  
// function to print a device address
void printAddress(DeviceAddress deviceAddress,byte temp)
{
  for (byte i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
   
    
  }
}

void odczytDHT()
{
  // sprawdzenie ile czasu upłyneło od odtatniego pomiaru 
  //z zabezpieczeniem przed przeładowaniem zmiennej millies() i rozpoczęciu od nowa
  if ( (   (millis()-TimeLastStart)>dht.getMinimumSamplingPeriod() ) || ( (millis()-TimeLastStart)<0) )
  {
    Serial.println("DHT info:");
    TimeLastStart=millis();//zapisanie czasu w kturym nastąpił pomiar
    //Serial.println(TimeLastStart);
    //Serial.println(millis());
    Serial.print(dht.getStatusString());
    int humidity = dht.getHumidity();
    int temperature = dht.getTemperature();
    //int(x);
    Serial.print("\t");
    lcd.setCursor(0,0);
    lcd.print(humidity, 1);
    //print2digits(humidity);
    lcd.print("% ");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.print("\t\t");
    //lcd.setCursor(0,1);
    lcd.print(temperature, 1);
    //print2digits(temperature);
    text=" ";
    lcd.print((char)0);lcd.print(text);
  }
}

void MenuSystem(byte pinButtom,byte pinButtom2, byte pinLed,byte pinRelay2)
{
  if (digitalRead(pinButtom) == LOW) {
    // turn LED on:
    delay(60);
    //Serial.println("Nacisnieto przycisk 1");
    P.OnPrzekaznik();
    lcd.backlight();
  } else {
    delay(60);
    // turn LED off:
    //digitalWrite(pinLed, HIGH);
    
  }

  if (digitalRead(pinButtom2) == LOW) {
    // turn LED on:
    delay(60);
    //Serial.println("Nacisnieto przycisk ____2");
    
    W.OnWentylator();
   
    lcd.backlight();
    // wyswietlenieCzasu();
     lcd.clear();
    //digitalWrite(pinRelay2, LOW);
  } else {
    delay(60);
    // turn LED off:
    //digitalWrite(pinRelay2, HIGH);
    
  }
  
  delay(160);
  
  if ((digitalRead(pinButtom) == LOW) && (digitalRead(pinButtom2) == LOW)){
  byte poziomMenu =0;
    delay(150);

    lcd.backlight();
    //Serial.println("Nacisnieto przycisk ____MENU");
    /*
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("MENU");
    lcd.setCursor(0,1);
    lcd.print(Menu[poziomMenu]);*/
    //Jestesmy w menu
    byte koniec_menu=0; 
    do{
     
    
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("MENU");
      lcd.setCursor(0,1);
      lcd.print(Menu[poziomMenu]);
    
      byte koniec=0;
      while(koniec<1){
      delay(350);
    
    
      if (digitalRead(pinButtom) == LOW){
      
        delay(160);
        poziomMenu++;
        if (poziomMenu==((sizeof(Menu)/sizeof(*Menu)))){
          poziomMenu=0;
        }
        lcd.setCursor(0,1);
        lcd.print(Menu[poziomMenu]);
        //Serial.print("MENU:");
        //Serial.println(Menu[poziomMenu]);
        
        
       
      
      }
      
      //nacisnieto przycisk zatwierdzajacy
     if (digitalRead(pinButtom2) == LOW){
      
        delay(160);
        switch( poziomMenu){
        
        case 1:{
        byte temp =  eep.read(9);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(temp);
        //Serial.print("MENU:Histereza ");
        //Serial.println(temp);
        byte tempKoniec=0;
        while(tempKoniec<1){
         
          
            if (digitalRead(pinButtom) == LOW){
              delay(250);
              temp++;
              //Serial.println(temp);
              lcd.setCursor(0,0);
              lcd.print(temp);
              if(temp>49){
                temp=1;
                lcd.clear();
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                eep.update(9,temp);
                //Serial.println("zapisano EEPROM");
                Wilgotnosc=temp;
                tempKoniec=1;
                //lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("zapisano        ");
                HisterezaTemp=temp;
                W.AktuHist(temp);
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }
        case 3:{ //czujnik PIR
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("ust.czasu czuwania");
        //Serial.print("MENU:ustawienia PIR");
        byte temp_min =  eep.read(11);
        Serial.println(temp_min);
       // byte temp_sek =  eep.read(12);
        delay(2000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(temp_min);
        lcd.setCursor(0,1);
        lcd.print("min");
        byte tempKoniec=0;
        while(tempKoniec<1){
         
          
            if (digitalRead(pinButtom) == LOW){
              delay(250);
              temp_min++;
              //Serial.println(temp_min);
              lcd.setCursor(0,0);
              lcd.print(temp_min);
              
              
              if(temp_min>59){
                temp_min=1;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(temp_min);
                lcd.setCursor(0,1);
                lcd.print("min");
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                eep.update(11,temp_min);
                //Serial.println("zapisano EEPROM");
                tempKoniec=1;
                lcd.setCursor(0,0);
                pause=temp_min*60000;
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }
        //Czujnik zmierzchu
        case 5:{ 
          
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("ust.czujnika zmierzchu");
        //Serial.print("MENU:ustawienia cujnika zmierzchu");
        byte temp_min =  PoziomZmierzchu;
        Serial.println(temp_min);
       // byte temp_sek =  eep.read(12);
        delay(2000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F.PoziomCiemnosciGraf());
        //lcd.print(temp_min);
        lcd.setCursor(0,1);
        lcd.print(F.PoziomCiemnosciGraf(temp_min));
        byte tempKoniec=0;
        while(tempKoniec<1){
         
          
            if (digitalRead(pinButtom) == LOW){
              delay(250);
              temp_min=temp_min+16;
              //Serial.println(temp_min);
              lcd.setCursor(0,0);
              lcd.print(F.PoziomCiemnosciGraf());
              lcd.setCursor(0,1);
              lcd.print(F.PoziomCiemnosciGraf(temp_min));
              
              
              if(temp_min>255){
                temp_min=1;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F.PoziomCiemnosciGraf());
                lcd.setCursor(0,1);
                lcd.print(F.PoziomCiemnosciGraf(temp_min));
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                eep.update(8,(temp_min));
                //Serial.println("zapisano EEPROM");
                tempKoniec=1;
                lcd.setCursor(0,0);
                PoziomZmierzchu=(temp_min);
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }
        case 4:{
        
        lcd.clear();
        dt = clock.getDateTime();
        lcd.setCursor(0,0); 
        lcd.print(dt.year);   lcd.print("-");
        lcd.print(dt.month);  lcd.print("-");
        lcd.print(dt.day);    lcd.print(" ");
        lcd.setCursor(0,1);
        lcd.print(dt.hour);   lcd.print(":");
        lcd.print(dt.minute); lcd.print(":");
        lcd.print(dt.second); lcd.print(" ");
        delay(1500);
        
        
        byte tempKoniec=0;
        byte tempKoniecUstawienCzasu=0;
        while (tempKoniecUstawienCzasu<1){
        //ustawiasz rok
        lcd.clear();
        lcd.setCursor(0,0);
        text="Rok";
        lcd.print(text);
        lcd.setCursor(0,1);
        lcd.print(dt.year);
        while(tempKoniec<1){
          if (digitalRead(pinButtom) == LOW){
              delay(250);
              dt.year++;
              lcd.setCursor(0,1);
              lcd.print(dt.year);
              if(dt.year>2030){
              dt.year=2014;
              
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                tempKoniec=1;
                lcd.setCursor(0,0);
                //if (!clock.isReady()){
                    
                clock.setDateTime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
                text="zapisano        ";
                lcd.print(text);
                /*}
                else{
                  text=" nie zapisano    ";
                  lcd.print(text);
                }*/
                delay(600);
                
              }
           
          
         }
        //ustawienia miesiącsa 
        lcd.clear();
        lcd.setCursor(0,0);
        text="Miesiac";
        lcd.print(text);
        lcd.setCursor(0,1);
        lcd.print(dt.month);
        tempKoniec=0;
        while(tempKoniec<1){
          if (digitalRead(pinButtom) == LOW){
              delay(250);
              dt.month++;
              lcd.setCursor(0,1);
              lcd.print(dt.month);
              if(dt.month>12){
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);   
              dt.month=1;
              lcd.print(dt.month);
              
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                tempKoniec=1;
                lcd.setCursor(0,0);
                clock.setDateTime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
                lcd.print("zapisano        ");
                delay(600);
                //koniec=1;
              }
           
          
         } 
        //ustawienia dnia
        
        lcd.clear();
        lcd.setCursor(0,0);
        text="Dzien";
        lcd.print(text);
        lcd.setCursor(0,1);
        lcd.print(dt.day);
        tempKoniec=0;
        while(tempKoniec<1){
          if (digitalRead(pinButtom) == LOW){
              delay(250);
              dt.day++;
              lcd.setCursor(0,1);
              lcd.print(dt.day);
              if(((dt.month==1||dt.month==3||dt.month==5||dt.month==7||dt.month==9||dt.month==11)&&(dt.day>31))||((dt.month==4||dt.month==6||dt.month==10||dt.month==12)&&(dt.day>30))||((dt.month==2)&&(dt.day>29))){
              dt.day=1;
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);
              lcd.print(dt.day); 
              
              }/*
              if((dt.month==4||dt.month==6||dt.month==10||dt.month==12)&&(dt.day>30)){
              dt.day=0;
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);
              lcd.print(dt.day); 
              
              }
              if((dt.month==2)&&(dt.day>29)){
              dt.day=1;
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);
              lcd.print(dt.day); 
              
              }*/
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                tempKoniec=1;
                lcd.setCursor(0,0);
                clock.setDateTime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
                lcd.print("zapisano        ");
                delay(600);
                //koniec=1;
              }
           
          
         }  
        //ustawienia Godzina
        
        lcd.clear();
        lcd.setCursor(0,0);
        text="Godzina";
        lcd.print(text);
        lcd.setCursor(0,1);
        lcd.print(dt.hour);
        tempKoniec=0;
        while(tempKoniec<1){
          if (digitalRead(pinButtom) == LOW){
              delay(250);
              dt.hour++;
              lcd.setCursor(0,1);
              lcd.print(dt.hour);
              if(dt.hour>24){
                
              dt.hour=1;
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);
              lcd.print(dt.hour); 
              
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                tempKoniec=1;
                lcd.setCursor(0,0);
                clock.setDateTime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
                lcd.print("zapisano        ");
                delay(600);
                //koniec=1;
              }
           
          
         }
        //ustawienia minuty
        
        lcd.clear();
        lcd.setCursor(0,0);
        text="Minuty";
        lcd.print(text);
        lcd.setCursor(0,1);
        lcd.print(dt.minute);
        byte tempKoniec=0;
        while(tempKoniec<1){
          if (digitalRead(pinButtom) == LOW){
              delay(250);
              dt.minute++;
              lcd.setCursor(0,1);
              lcd.print(dt.minute);
              if(dt.minute>59){
              dt.minute=0;
              lcd.setCursor(0,1);
              text="  ";
              lcd.print(text);
              lcd.setCursor(0,1);
              lcd.print(dt.minute);  
              
              
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                tempKoniec=1;
                lcd.setCursor(0,0);
                clock.setDateTime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
                tempKoniecUstawienCzasu=1;
                wyswietlenieCzasu();
              }
           
          
         }      
        
        }
        break;
        }
        case 2:{
        byte temp =  eep.read(10);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(temp);
        //Serial.print("MENU:wilgotnosc ");
        //Serial.println(temp);
        byte tempKoniec=0;
        while(tempKoniec<1){
         
          
            if (digitalRead(pinButtom) == LOW){
              delay(250);
              temp++;
              //Serial.println(temp);
              lcd.setCursor(0,0);
              lcd.print(temp);
              if(temp>98){
              temp=20;
              lcd.clear();
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                eep.update(10,temp);
                //Serial.println("zapisano EEPROM");
                Wilgotnosc=temp;
                tempKoniec=1;
                //lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }/*
        //stan baterii RTC DS1307
        case 5:{
          float batteryVoltageRead = analogRead (A0);
          byte tempKoniec=0;
          lcd.clear();
          lcd.setCursor(0,0);
          text="Stan naladawania baterii";
          lcd.print(text);
          float batteryVoltage;  
          while(tempKoniec<1){
            batteryVoltage =float( batteryVoltageRead * (5/1023.) );  // format might need some tweaking
            lcd.setCursor(0,1);
            lcd.print(batteryVoltage,2);
            text=" V - ";
            lcd.print(text);
            Serial.println (batteryVoltage, 2);  // same here, not sure how to specify how many decimal points are shown
            batteryVoltage = (batteryVoltage/3.6)*100;
            lcd.print(batteryVoltage, 1);
            text=" %";
            lcd.print(text);
            if (digitalRead(pinButtom2) == LOW){
                delay(250);
                koniec=1;
                tempKoniec=1;
               
            }
          }
          break;
        }*/
        //koniec wyjscie z menu
        case 6:{
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("wyjscie         ");
        //Serial.print("MENU:koniec");
        koniec_menu=1;
        koniec=1;
        delay(2000);
        lcd.clear();
        break;
        }
        default:
          //koniec=1;
        break;
        }
     }
      
      }  
    }while( koniec_menu!=1);
  
  }
  
}
/*
unsigned long CzasWykonania( void( * wsk_na_funkcje )() )
{
    wsk_na_funkcje();
    return zwracana_wartosc; // albo nic nie zwracać
   
}
*/



void wyswietlenieCzasu()
{
  dt = clock.getDateTime();

  // For leading zero look to DS1307_dateformat example

  Serial.print("Raw data: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");

  
  lcd.setCursor(0,0); 
  lcd.print(dt.year);   lcd.print("-");
  lcd.print(dt.month);  lcd.print("-");
  lcd.print(dt.day);    lcd.print(" ");
  lcd.setCursor(0,1);
  lcd.print(dt.hour);   lcd.print(":");
  lcd.print(dt.minute);// lcd.print(":");
  //lcd.print(dt.second); lcd.print(" ");

  delay(5000);
}
//funkcja czujnika PIR
void loopPIR()
{

     if(digitalRead(pirPin) == HIGH){
      
      digitalWrite(pinLED, HIGH);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         Serial.println("---");
         Serial.print("motion detected at ");
         lcd.backlight();
         if(F.SprawdzajStanPIR() ){
          P.OnPrzekaznik(1);
          }
         else{
          if(F.SprawdzajCiemnosc(PoziomZmierzchu)){
            P.OnPrzekaznik(1);
            F.UstawStanPIR(1);
          }
         }
         
         Serial.print(millis()/1000);
         Serial.println(" sec"); 
         delay(50);
       
         }         
         takeLowTime = true;
                
       }

     if(digitalRead(pirPin) == LOW){ 
            
      digitalWrite(pinLED, LOW);  //the led visualizes the sensors output pin state

       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        F.UstawStanPIR(1);
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           Serial.print("motion ended at ");      //output
           //lcd.setCursor(0, 1); 
         
           //lcd.print("ended");
           Serial.print((millis() - pause)/1000);
          Serial.println(" sec");
           
           lcd.noBacklight();
           P.OnPrzekaznik(0);
           F.UstawStanPIR(0);
           delay(50);
           
           
           }
           
       }
  }
