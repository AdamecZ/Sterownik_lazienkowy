#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DS1307.h>

// zegar i EEPROM układ Tiny RTC DS1307
DS1307 clock;
RTCDateTime dt;


 // Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9

char* Menu[]={"temperatura","wilgotnosc","wentylator","koniec"};
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
int numberOfDevices; // Number of temperature devices found
byte pirPin = 3; //pin czujnika ruchu PIR

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
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
byte calibrationTime = 30;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;  

// Timer
unsigned long TimeLastStart=0 ;

void setup(void)
{
  Serial.begin(9600);
  
   // Initialize DS1307
  Serial.println("Initialize DS1307");
  clock.begin();

  // If date not set
  if (!clock.isReady())
  {
    // Set sketch compiling time
    clock.setDateTime(__DATE__, __TIME__);
  }

   // Fill memory
    // Read a memory
  char tmp[16];
  Serial.println("Reading memory");
  Serial.println();
  Serial.print("        ");

  for (byte i = 0; i < 16; i++)
  {
      sprintf(tmp, "0x%.2X ", i, 2);
      Serial.print(tmp);
  }

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------");

  for (byte j = 0; j < 4; j++)
  {
    sprintf(tmp, " 0x%.2X : ", (j*16), 2);
 
    Serial.print(tmp);
 
    for (byte i = 0; i < 16; i++)
    {
      if ((j*16 + i) > 55)
      {
        break;
      }

      sprintf(tmp, "0x%.2X ", clock.readByte(j*16 + i), 2);

      Serial.print(tmp);
    }

    Serial.println();
  }
  

//---------------------
   pinMode (4,INPUT_PULLUP);// pin 4 dla przycisku podłaczonego do GND
   pinMode (8,INPUT_PULLUP);// pin 8 dla przycisku podłaczonego do GND
   pinMode(5, OUTPUT);//pin przekaźnika automat z PIR
   pinMode(7,OUTPUT);//pin przekaźnika przycisku  
   pinMode(pirPin, INPUT);//sekcja czujnika PIR
   digitalWrite(5,HIGH);//ustalenie stanu poczatkowego
   digitalWrite(7, HIGH);
   
   dht.setup(6); // data pin 6
   //Serial.println(dht.getModel());
   Serial.println("Inicjalizacja wyświetlacxza LCD...");
   lcd.begin(16,2);                      // initialize the lcd 
   lcd.backlight();
   
  //wyswietlenie czasu 
  wyswietlenieCzasu();

  //wyswietlenie tytuł
  lcd.home();
  
  lcd.print("Sterownik lazienkowy");
  lcd.setCursor(0, 1); 
  lcd.print("v1.0 29.02.2016");
  delay(2000);
  lcd.clear();
  
  //pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);
  lcd.clear();
  lcd.home();
  //give the sensor some time to calibrate
  Serial.println("calibrating sensor ");
  lcd.print("kalibracja PIR");
  
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      lcd.setCursor(i,1);
      lcd.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
  // start serial port
  
  
  Serial.println("Dallas Temperature IC Control Library Demo");
  // locate devices on the bus
  Serial.print("Locating devices...");

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
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
  {
    Serial.print("Found device ");
    Serial.print(i, DEC);
    Serial.print(" with address: ");
    printAddress(tempDeviceAddress);
    Serial.println();
    
    Serial.print("Setting resolution to ");
    Serial.println(TEMPERATURE_PRECISION, DEC);
    
    // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    
     Serial.print("Resolution actually set to: ");
    Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
    Serial.println();
  }else{
    Serial.print("Found ghost device at ");
    Serial.print(i, DEC);
    Serial.print(" but could not detect address. Check power and cabling");
  }
  }
  lcd.clear();
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

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
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
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.print("\t\t");
  }
}
class Przycisk
{
  private:
  unsigned long CzasStartu;
  byte pin;
  
  public:
  void begin(byte pinButton)
  {
    pin=pinButton;
  }
  
};
void buttom(byte pinButtom,byte pinButtom2, byte pinLed)
{
  if (digitalRead(pinButtom) == LOW) {
    // turn LED on:
    delay(60);
    Serial.println("Nacisnieto przycisk 1");
    digitalWrite(pinLed, LOW);
  } else {
    delay(60);
    // turn LED off:
    digitalWrite(pinLed, HIGH);
    
  }

  if (digitalRead(pinButtom2) == LOW) {
    // turn LED on:
    delay(60);
    Serial.println("Nacisnieto przycisk ____2");
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
    Serial.println("Nacisnieto przycisk ____MENU");
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("MENU");
    //sizeof(*Menu)
    byte koniec=0;
    while(koniec<1){
    delay(350);
    Serial.println(sizeof(Menu)/sizeof(*Menu));
    Serial.println(poziomMenu);
    
    if (digitalRead(pinButtom) == LOW){
      
        delay(160);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(Menu[poziomMenu]);
        Serial.print("MENU:");
        Serial.println(Menu[poziomMenu]);
        
        if (poziomMenu==3){
          poziomMenu=0;
        }
        else poziomMenu++;
      
      }
      
      //nacisnieto przycisk zatwierdzajacy
     if (digitalRead(pinButtom2) == LOW){
      
        delay(160);
        switch( poziomMenu )
        {
        case 1:{
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("wartosc 1");
        Serial.print("MENU:wartosc1");
        break;
        }
        case 2:{
          lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("wartosc 2");
        Serial.print("MENU:wartosc2");
        break;
        }
        case 3:{
        byte temp =  clock.readByte(10);
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(temp);
        Serial.print("MENU:wartosc3 ");
        Serial.println(temp);
        byte tempKoniec=0;
        while(tempKoniec<1){
         
          
            if (digitalRead(pinButtom2) == LOW){
              delay(250);
              temp++;
              Serial.println(temp);
              lcd.setCursor(0,0);
              lcd.print(temp);
              if(temp>99) temp=0;
            }
              if (digitalRead(pinButtom) == LOW){
                delay(250);
                clock.writeByte(10,temp);
                Serial.println("zapisano EEPROM");
                tempKoniec=1;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("zapisano");
                delay(600);
                koniec=1;
              }
           
          
         }
        break;
        }
        
        case 0:{
          lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("wyjscie");
        Serial.print("MENU:koniec");
        koniec=1;
        break;
        }
        default:
          //koniec=1;
        break;
        }/*
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(Menu[poziomMenu]);
        Serial.print("MENU:");
        Serial.println(Menu[poziomMenu]);
        */
      
      } 
    }
  
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
  //     digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         Serial.println("---");
         Serial.print("motion detected at ");
         Serial.print(millis()/1000);
         Serial.println(" sec"); 
         delay(50);
         lcd.home();
         lcd.print("RUCH");
         digitalWrite(5, LOW);
         }         
         takeLowTime = true;
                
       }

     if(digitalRead(pirPin) == LOW){ 
            
//       digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state

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
           lcd.home();
           lcd.print("    ");
           }
           
       }
  }
