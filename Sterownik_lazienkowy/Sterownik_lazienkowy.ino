#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DS1307.h>
#include <AT24Cxx.h>

// zegar i EEPROM układ Tiny RTC DS1307
DS1307 clock;
RTCDateTime dt;

AT24Cxx eep(0x50, 32);


 // Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9
#define pinDHT 6//pin DHT22
#define pirPin  3 //pin czujnika ruchu PIR
#define ledPin1 7//pin 

char* Menu[]={"temperatura     ","wilgotnosc      ","wentylator      ","czujnik ruchu   ","koniec          "};
// inicjalizacja DHT 22 czujnik wilgoci
DHT dht;

// inicjalizacjia LCD
LiquidCrystal_I2C lcd(0x20,4,5,6,0,1,2,3,7,NEGATIVE);  // set the LCD address to 0x20 for a 16 chars and 2 line display

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempDeviceAddress;
DeviceAddress tempDeviceAddressEEPROM[]={0};
byte numberOfDevices; // Number of temperature devices found


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
long unsigned int pause = 0;  

boolean lockLow = true;
boolean takeLowTime;  

// Timer
unsigned long TimeLastStart=0 ;
//-------------------------------------------------------------
void setup(void)
{
  Serial.begin(9600);
  
   // Initialize DS1307
  //Serial.println("Initialize DS1307");
  clock.begin();

  // If date not set
  if (!clock.isReady())
  {
    // Set sketch compiling time
    clock.setDateTime(__DATE__, __TIME__);
  }
  //wyswietlenie czasu 
  wyswietlenieCzasu();

//---------------------
   pinMode (4,INPUT_PULLUP);// pin 4 dla przycisku podłaczonego do GND
   pinMode (8,INPUT_PULLUP);// pin 8 dla przycisku podłaczonego do GND
   pinMode(5, OUTPUT);//pin przekaźnika automat z PIR
   pinMode(7,OUTPUT);//pin przekaźnika przycisku  
   pinMode(pirPin, INPUT);//sekcja czujnika PIR
   digitalWrite(5,HIGH);//ustalenie stanu poczatkowego
   digitalWrite(7, HIGH);
   
   dht.setup(pinDHT); // data pin 6
   //Serial.println(dht.getModel());
   //Serial.println("Inicjalizacja wyświetlacxza LCD...");
   lcd.begin(16,2);                      // initialize the lcd 
   lcd.backlight();
   
   wyswietlenieTytulu(); 
    
  //pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);
  lcd.clear();
  lcd.home();
  //give the sensor some time to calibrate
  //Serial.println("calibrating sensor ");
  lcd.print("kalibracja PIR");
  
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
    //Serial.println(pause);
    delay(50);
  
  
  
  //Serial.println("Dallas Temperature IC Control Library Demo");
  // locate devices on the bus
  //Serial.print("Locating devices...");

  sensors.begin();
  Serial.print("Found ");
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  // Loop through each device, print out address
  for(byte i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.println("Odczytano adres termometru z pamięci EEPROM");
      for(byte temp=0;temp<8;temp++)
      {
        //odczytanie z EEPROM adresów DS18B20
        tempDeviceAddressEEPROM [i][temp]=eep.read((8*i)+(2000+temp)+i);
        Serial.print(tempDeviceAddressEEPROM [i][temp],HEX);
      }
      if (strcmp(tempDeviceAddress,tempDeviceAddressEEPROM[i]) == 0)
      {
        Serial.println("termometr jest juz zapisany w EEPROM");   
      }
      else
      {
        Serial.println("Dodawanie nowego termometru");
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
}



void loop(void)
{ 
  buttom(4,8,7);
  odczytDHT();
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  
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
    float temperatura=GetToPrintTemperature(tempDeviceAddress); // Use a simple function to print out the data
    
    lcd.setCursor(10,i);
    lcd.print(temperatura);
    
  } 
  //else ghost device! Check your power requirements and cabling
  
  }
  loopPIR();
}

// function to print the temperature for a device
float GetToPrintTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
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
   // DaneDS18B20 [temp][i]=deviceAddress[i];
    //eep.update(11+i,deviceAddress[i]);
   // Serial.print(eep.read(11+i),HEX);
    
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
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.print("\t\t");
  }
}

void buttom(byte pinButtom,byte pinButtom2, byte pinLed)
{
  if (digitalRead(pinButtom) == LOW) {
    // turn LED on:
    delay(60);
    //Serial.println("Nacisnieto przycisk 1");
    digitalWrite(pinLed, LOW);
  } else {
    delay(60);
    // turn LED off:
    //digitalWrite(pinLed, HIGH);
    
  }

  if (digitalRead(pinButtom2) == LOW) {
    // turn LED on:
    delay(60);
    //Serial.println("Nacisnieto przycisk ____2");
    digitalWrite(pinLed, LOW);
  } else {
    delay(60);
    // turn LED off:
    digitalWrite(pinLed, HIGH);
    
  }
  
  delay(160);
  byte poziomMenu =0;
  if ((digitalRead(pinButtom) == LOW) && (digitalRead(pinButtom2) == LOW)){
  
    delay(150);

    
    //Serial.println("Nacisnieto przycisk ____MENU");
    
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
        
        lcd.setCursor(0,1);
        lcd.print(Menu[poziomMenu]);
        //Serial.print("MENU:");
        //Serial.println(Menu[poziomMenu]);
        
        if (poziomMenu==(sizeof(Menu)/sizeof(*Menu))){
          poziomMenu=0;
        }
        poziomMenu++;
      
      }
      
      //nacisnieto przycisk zatwierdzajacy
     if (digitalRead(pinButtom2) == LOW){
      
        delay(160);
        switch( poziomMenu-1 ){
        
        case 1:{
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("wartosc 1");
        //Serial.print("MENU:wartosc1");
        koniec=1;
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
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
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
                temp=0;
                lcd.clear();
              }
            }
              if (digitalRead(pinButtom2) == LOW){
                delay(250);
                eep.update(10,temp);
                //Serial.println("zapisano EEPROM");
                tempKoniec=1;
                //lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("zapisano        ");
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }
        
        case 4:{
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
  lcd.print(dt.minute); lcd.print(":");
  lcd.print(dt.second); lcd.println("");

  delay(5000);
}
//funkcja czujnika PIR
void loopPIR()
{

     if(digitalRead(pirPin) == HIGH){
      lcd.backlight();
      //digitalWrite(ledPin1, HIGH);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         Serial.println("---");
         Serial.print("motion detected at ");
         Serial.print(millis()/1000);
         Serial.println(" sec"); 
         delay(50);
         
         digitalWrite(5, LOW);
         }         
         takeLowTime = true;
                
       }

     if(digitalRead(pirPin) == LOW){ 
            
     // digitalWrite(ledPin1, LOW);  //the led visualizes the sensors output pin state

       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           Serial.print("motion ended at ");      //output
           Serial.print((millis() - pause)/1000);
           Serial.println(" sec");
           digitalWrite(5,HIGH);
           delay(50);
           lcd.noBacklight();
           
           }
           
       }
  }
