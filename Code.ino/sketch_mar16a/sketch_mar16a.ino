#define BLYNK_TEMPLATE_ID "TMPL3lbBH3wS7"
#define BLYNK_TEMPLATE_NAME "esp 2"
#define BLYNK_AUTH_TOKEN "YsE9uoDbRSzcLhJ6aT3v2s4uBPwkkA3e"
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Cosmic Dragon";        // Enter your Wifi Username
char pass[] = "Hello World";  // Enter your Wifi password


////////////////////////// DHT11 //////////////////////////////////////
int buzzer = D2;
const int flowSensorPin = D3;  // Flow sensor signal pin

// Flow rate variables


const int rainSensorPin = D4;

#include <DHT.h>
int DHTPIN = D1;       // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT 11
DHT dht(DHTPIN, DHTTYPE);
float a;
////////////////////
int TRIGGER_PIN = D7;  // Arduino pin connected to the trigger pin of the ultrasonic sensor
int ECHO_PIN = D6;
///////////////////////
const int flowPin = 0;  // The pin the sensor is connected to
unsigned int flowFrequency;
unsigned long totalMilliLitres;
float flowRate;
unsigned long oldTime;

void IRAM_ATTR flowPulse() {  // Place ISR in IRAM
  flowFrequency++;            // Increment the pulse count
}
/////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  // Set the pin as input with pull-up resistor

pinMode(rainSensorPin,INPUT);
  pinMode(buzzer, OUTPUT);
  dht.begin();
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(flowPin), flowPulse, RISING);  // Attach interrupt to the pin
  oldTime = millis();
  pinMode(flowPin, INPUT_PULLUP);
}

void loop() {

  Blynk.run();

 

 


  flow();

 








  float humidity = dht.readHumidity();
  float temperature_C = dht.readTemperature();


  Blynk.virtualWrite(V1, temperature_C);
  Blynk.virtualWrite(V2, humidity);







  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure the duration of the echo pulse
  unsigned long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in centimeters
  float distance_cm = duration * 0.034 / 2;
  float distance = 1000 - distance_cm;
  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  Blynk.virtualWrite(V3, distance);
Blynk.virtualWrite(V0, distance_cm);
  if(distance_cm<=5){
 digitalWrite(buzzer, 1);
   
    delay(100);
    digitalWrite(buzzer,0);
    delay(200);

  }
  if(distance_cm>5){
 digitalWrite(buzzer,0);

  }}



void flow() {
  if ((millis() - oldTime) > 1000) {                                     // Every second, calculate flow rate
    detachInterrupt(digitalPinToInterrupt(flowPin));                     // Disable the interrupt while calculating
    flowRate = (1000.0 / (millis() - oldTime)) * flowFrequency / 7.5;    // Calculate flow rate in L/min
    oldTime = millis();                                                  // Update oldTime
    totalMilliLitres += (flowFrequency / 7.5);                           // Update total flow in milliliters
    flowFrequency = 0;                                                   // Reset flow frequency counter
    attachInterrupt(digitalPinToInterrupt(flowPin), flowPulse, RISING);  // Re-enable the interrupt
  }
  // Print flow rate and total flow
  Serial.print("Flow rate: ");
  Serial.print(flowRate);
  Serial.print(" L/min\t");
  Serial.print("Total flow: ");
  Serial.print(totalMilliLitres);
  Serial.println(" ml");
  Blynk.virtualWrite(V4, flowRate);
  Blynk.virtualWrite(V5, totalMilliLitres);
}