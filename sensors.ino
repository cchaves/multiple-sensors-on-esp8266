// Information available at Ubidot.com
// Using Ubidots library 

//                     PIN    GPIO
//static const uint8_t D0   = 16;  //nodemcu 2nd LED 
//static const uint8_t D1   = 5;   //Safe to use
//static const uint8_t D2   = 4;   //nodemcu LED default built in LED do not use as a GPIO data since the LED can cause voltage drops
//static const uint8_t D3   = 0;   // Safe to use low during download
//static const uint8_t D4   = 2;   // U1TXD line
//static const uint8_t D5   = 14;  // Safe to use
//static const uint8_t D6   = 12;  // Safe to use
//static const uint8_t D7   = 13;  // Safe to use
//static const uint8_t D8   = 15;  // Safe to use low during power on
//static const uint8_t D9   = 3; // Dont use since it is R0RXD
//static const uint8_t D10  = 1;  // High during power on
//GPIO pins 2,4,5,12,13,14,15 and 16 as input


/****************************************
   Include Libraries
 ****************************************/
#include "UbidotsESPMQTT.h" 
#include <DHT.h>; //Library used for DHT sensors such DHT22 temp humudity

/****************************************
   Define Constants
 ****************************************/

int myrestart = 300;  
int hum; //Stores humidity value
int dhtpwr = 14; // used as VCC on DHT22 to toggle on off for better results our attch to external VCC 3-5V.
int dhtsetupT = 2010;//Minimun DHT22 setup time 2 sec or 2K mill sec
float temp; //Stores temperature
int adcValue; //Stores data for Analog watter sensor
float heatI; //Calcuated Heat Index by theDHT lib
#define DHTPIN 15 //Data pin for DHT 22   avoid pin 2 or D4 since D4 is attached to the build in LED
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE); /// Init DHT sensor

#define MQTT_CLIENT_NAME "ESP8266"
#define TOKEN "BBFF-I....." // Your Ubidots TOKEN
#define WIFINAME "ou" //Your SSID
#define WIFIPASS "Per" // Your Wifi Password

#define sensor    A0        // Hook water sensor to pin A0 of NODEMCU module
#define RADAR     D1       // Radar movement senson 
#define LED       D0       // Led in NodeMCU/WeMos D1 Mini at pin GPIO2 (D4)
#define BRIGHT    150      // Max led intensity (1-500)
#define INHALE    900     // Inhalation time in milliseconwatteds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      10000    // Rest Between Inhalations.

Ubidots client(TOKEN);

/****************************************
   Auxiliar Functions
 ****************************************/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/****************************************
   Main Functions
 ****************************************/

void setup() {
  // put your setup code here, to run once:
  client.ubidotsSetBroker("things.ubidots.com");
  Serial.setRxBufferSize(400); //Added a larger Rx buffer for serial communications
  Serial.begin(115200);
  pinMode(sensor, INPUT);   // Analog pin setup for read
  pinMode(RADAR, INPUT);    // radar sensor to read status
  pinMode(LED, OUTPUT);     // LED pin as output.
  client.setDebug(true);    // Pass a true or false bool value to activate debug messages
 
  client.wifiConnection(WIFINAME, WIFIPASS);
  client.begin(callback);

  pinMode(dhtpwr,OUTPUT); //D5 pin 14 power pin for DHT sensor more stable power than VCC 3v
  Serial.flush(); // force serial outpout to avoid hanged up serial buffer

 
}

void loop() {
  // Make sure the water sensor is connected to your WiFi network. If not try to reconnect. 
  if (!client.connected()) {
    client.reconnect();
  }



  pinMode(dhtpwr,OUTPUT); //D5 pin 14 power pin for DHT sensor more stable power than VCC 3v
  digitalWrite(14,HIGH); // D5 set high to power on sensor
  dht.begin(); //DHT sensor on sensor function
  Serial.flush(); // force serial outpout to avoid hanged up serial buffer
  delay(dhtsetupT); // Wait to get started on sensor data  the HDT sensors are slow so they need a delay to get data about 2 secs

  
  for (int i = 1; i < BRIGHT; i++) {
    digitalWrite(LED, LOW);            // turn the LED on.
    delayMicroseconds(i * 10);         // wait
    digitalWrite(LED, HIGH);           // turn the LED off.
    delayMicroseconds(PULSE - i * 10); // wait
    delay(0);                          // to prevent watchdog firing.
  }
  //ramp decreasing intensity, Exhalation (half time):
  for (int i = BRIGHT - 1; i > 0; i--) {
  digitalWrite(LED, LOW);            // turn the LED on.
    delayMicroseconds(i * 10);         // wait
    digitalWrite(LED, HIGH);           // turn the LED off.
    delayMicroseconds(PULSE - i * 10); // wait
    i--;
    delay(0);                          // to prevent watchdog firing.
  }

 
    hum = dht.readHumidity();                //Read Humidity
    temp= dht.readTemperature(true);         //Read Temp in F  val true  
    heatI = dht.computeHeatIndex(temp, hum); //Calcualte Heat Index
        
  adcValue = analogRead(sensor);    // Read the ADC channel
  int moveyes = 25;
  int moveon = digitalRead(RADAR);
  unsigned long SEC = 1000L;
  unsigned long MIN = SEC * 60;
  
  if (moveon==1) { moveyes = 50;
  }
  else { moveyes = 25;
  }

  digitalWrite(dhtpwr,LOW); //GPIO set LOW to power OFF sensor DHT provides more stable power to ESP during WiFi data send

  client.add("h2o", adcValue);     // Variable for the water heater sensor assigned the ADC value. This will show up in Ubidots within the water-sensor device
  client.add("hum",hum);
  client.add("temp",temp);
  client.add("radar",moveyes);   // Publish 25 only if movement found
  
  client.ubidotsPublish("chav_sensor");  // Device name for Ubidots. Make sure Ubidots is setup prior to loading and running this application. 

  client.loop();

  delay(MIN);                  // take a rest  it is needed for Ubidots since they only allow data every few seconds ...
  delay(MIN);
  delay(MIN);
  
// restart the ESP after a few for estability
//    Serial.print("ESP Restart Counter ");
//    Serial.println(myrestart);
 
  if(myrestart==0){
    Serial.println("Reset..");
    ESP.restart();
  }
 
  --myrestart; // decrease counter to restart

}
