// Temerature - Humidity and Water Sensor that will text me if the sensor goes off.
// Information available at Ubidot.com
// Created by jnissen using Ubidots library 
// May 16th, 2017 - Added commentys by chaves
//static const uint8_t D0   = 16;  nodemcu LED 
//static const uint8_t D1   = 5;
//static const uint8_t D2   = 4;   nodemcu LED default built in LED do not use as a GPIO data since the LED can cause voltage drops
//static const uint8_t D3   = 0;
//static const uint8_t D4   = 2;
//static const uint8_t D5   = 14;
//static const uint8_t D6   = 12;
//static const uint8_t D7   = 13;
//static const uint8_t D8   = 15;
//static const uint8_t D9   = 3;
//static const uint8_t D10  = 1;



/****************************************
   Include Libraries
 ****************************************/
#include "UbidotsESPMQTT.h" //the library uses PubSubClient.h if you have mutiple sensors you need to change the mesage sie on the PubsubClient from 128 to 256 or larger
#include <DHT.h>; //Library used for DHT sensors such DHT22 temp humudity
/****************************************
   Define Constants
 ****************************************/

int hum; //Stores humidity value
int dhtpwr = 14; // used as VCC on DHT22 to toggle on off for better results our attch to external VCC 3-5V.
int dhtsetupT = 2010;//Minimun DHT22 setup time 2 sec or 2K mill sec
float temp; //Stores temperature
int adcValue; //Stores data for Analog watter sensor
float heatI; //Calcuated Heat Index by theDHT lib
#define DHTPIN 15 //Data pin for DHT 22   avoid pin 2 or D4 since D4 is attached to the build in LED
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE); /// Init DHT sensor

#define TOKEN "A1E-kYKuNNClfZkuC0GKT2WyLsaw4qFA9V" // Your Ubidots TOKEN
#define WIFINAME "ourHome" //Your SSID
#define WIFIPASS "Perfectpond217" // Your Wifi Password

#define sensor    A0       // Hook water sensor to pin A0 of NODEMCU module
#define LED       D0       // Led in NodeMCU/WeMos D1 Mini at pin GPIO2 (D4)
#define BRIGHT    150      // Max led intensity (1-500)
#define INHALE    900     // Inhalation time in milliseconwatteds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      15000    // Rest Between Inhalations.

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
  Serial.setRxBufferSize(400); //Added a larger Rx buffer for serial communications
  Serial.begin(115200);
  pinMode(sensor, INPUT);   // Analog pin setup for read
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

  // A fancy delay routine! Makes the LED appear to "breathe" by slowly ramping up and down. This is to show you the water sensor is working 
  // by slowly making the LED on the board glow from dim to bright and back dim. It's not really needed but it's a bit of fun to watch.
  // If you don't add some delay the water sensor will attempt to "log" data often and your free Ubidots account may be disabled. This 
  // delay is intended to create a new data point once every 15 seconds or so. 

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

 
  
  // Publish values of ADC0 water sensor. Water will cause the voltage to rise and the ADC will read this as a higher value.
  // Once the value is read the NODEMCU will publish it to UBIDOTS. The Node MCU does not care what the reading is. It only reports it.
  // If below trigger value the text message will NOT be delivered. Above trigger it's sent.

    hum = dht.readHumidity();                //Read Humidity
    temp= dht.readTemperature(true);         //Read Temp in F  val true  
    heatI = dht.computeHeatIndex(temp, hum); //Calcualte Heat Index
        
  adcValue = analogRead(sensor);    // Read the ADC channel
  client.add("h2o", adcValue);     // Variable for the water heater sensor assigned the ADC value. This will show up in Ubidots within the water-sensor device
  client.add("hum",hum);
  client.add("temp",temp);
  client.add("HeatIndex",heatI);
  client.ubidotsPublish("chav_sensor");  // Device name for Ubidots. Make sure Ubidots is setup prior to loading and running this application. 
  client.loop();

  digitalWrite(dhtpwr,LOW); // D5 set LOW to power OFF sensor
  delay(REST);                  // take a rest...

}
