/*****************************************************************************************************************
 *Developer: Salvatore Nicosia                                                                                   *                              
 *Description:                                                                                                   *
 *              The following code allows the control of the smart blinds through the blynk app and the buttons. *
 *              Some of the parts in this code are still a working progress to enable Alexa and Google Assistant.*
 *                                                                                                               *
 *              IMPORTANT! In order to connect the Wemos D1 to the WiFi the sections Blynk Setup and WiFi        * 
 *              parameters need to be changed with your WiFi SSID and Password.                                  *
 *****************************************************************************************************************/

#define BLYNK_PRINT Serial

#include <BlynkSimpleEsp8266.h>
#include <SPI.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <functional>
#include "switch.h"
#include "UpnpBroadcastResponder.h"
#include "CallbackFunction.h"

// prototypes
void move_motors(int potValue);
void blink_red_led(int timeDelay, int noTimes);
void blink_green_led(int timeDelay, int noTimes);
boolean connectWifi();
void MQTT_connect();

/****************************** Blynk Setup **********************************/
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "-------------------------";     // Your Authorization Token

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "------------";
char pass[] = "------------";
/**************************** END Blynk Setup ********************************/

// Change WiFi parameters before flashing
#define WiFi_SSID       "------------"         // Your SSID
#define WiFi_PASS       "------------"         // Your password

boolean wifiConnected = false;

UpnpBroadcastResponder upnpBroadcastResponder;

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "snicosia"            // Replace it with your username
#define AIO_KEY         "ad30480c676f4963aad3aec25c87eb92"   // Replace with your Project Auth Key

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/******************************* Feeds ***************************************/

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe Apri = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/Apri"); // FeedName
Adafruit_MQTT_Subscribe Chiudi = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Chiudi");

unsigned long previousMillis = 0;

//Wemos LED Pins
int LED1 = D5; //Green LED
int LED2 = D6; //Red LED

//Wemos Buttons Pin
int DownButtonPin = D2;
int UpButtonPin = D1;

int CurrentState; // Current State of The Blinds 0 = Closed 1 = Open
int DownTime = (EEPROM.read(30) * 100); // Time in ms to close blinds
int UpTime = (EEPROM.read(4) * 100); // Time in ms to close blinds

int NewDownTime;
int NewUpTime;
int SetTestEEPROM;
int RecTestEEPROM;
float UpDownRatio = 1.546583; //Ratio between up and down time

//Wemos PWM Speed Control Pins：
int E1 = D3;
int M1 = D4;

// LED Pins
const int ledPin = D5;
const int RedLedPin = D6;
const int inputPin = D2;

boolean LED1State = false;
boolean LED2State = false;

long buttonTimer = 0;
long longPressTime = 500;

boolean DownButtonActive = false;
boolean DownLongPressActive = false;

boolean UpButtonActive = false;
boolean UpLongPressActive = false;

String s;    //to store incoming text ingredient
BLYNK_WRITE(V0)
{
  s = param.asStr();
  Serial.print(s); //string sent by Blynk app will be printed on Serial Monitor
  if (s == "apri tenda")
  {
    buttonTimer = millis();
    move_motors(1024); // Move motors up
    digitalWrite(LED1, HIGH); //turn on green led while moving
  } else if (s == "chiudi tenda")
  {
    buttonTimer = millis();
    move_motors(0); // Move motors down
    digitalWrite(LED2, HIGH); //turn on green led while moving
  }
  else {
    Serial.print("Say apri or chiudi");
  }
}

void setup() {
  Serial.begin(115200);

  Serial.print("Blinds Controller");

  pinMode(UpButtonPin, OUTPUT);
  pinMode(DownButtonPin, OUTPUT);

  // Initialize wifi connection
  wifiConnected = connectWifi();

  // Initialize Blynk App
  Blynk.begin(auth, ssid, pass);

  // Initialize MQTT
  //MQTT_connect();

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Apri);
  mqtt.subscribe(&Chiudi);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(DownButtonPin, INPUT);

  RecTestEEPROM = EEPROM.read(2);
  Serial.println(RecTestEEPROM);

  // Check if data already stored to EEPROM if not assume blinds are closed
  int GetCurrentState;
  GetCurrentState = EEPROM.read(0);
  if (GetCurrentState < 0) // Not yet stored in EEPROM, assume blind is closed, and set it to closed
  {
    EEPROM.write(0, CurrentState);
    Serial.println("Data missing, blinds assumed closed");
  } else // Else load the current state from EEPROM
  {
    CurrentState = GetCurrentState;
  }

  // Get Down Time
  GetCurrentState = EEPROM.read(30);
  if (GetCurrentState <= 0) // Not yet stored in EEPROM, assume zero time so no accidents occur
  {
    EEPROM.write(30, 0);
    Serial.println("Down Time missing, please calibrate");
    // Blink red led 2 times
    blink_red_led(100, 2);
  } else // Else load the current state from EEPROM
  {
    DownTime = GetCurrentState;
  }

  // Get Up Time
  GetCurrentState = EEPROM.read(4);
  if (GetCurrentState <= 0) // Not yet stored in EEPROM, assume zero time so no accidents occur
  {
    EEPROM.write(4, 0);
    Serial.println("Up Time missing, please calibrate");
    // Blink red led 3 times
    blink_red_led(100, 3);
  } else // Else load the current state from EEPROM
  {
    UpTime = GetCurrentState;
  }

  //Default time settings
  UpTime = 27000;
  DownTime = 18700;

  //Write some status
  Serial.print("Blind State:");
  if (CurrentState == 1) {
    Serial.println(" Open");
    digitalWrite(LED1, HIGH); //Turn on the Green LED
  } else {
    Serial.println(" Closed");

  }
  Serial.print("Up Time (ms): ");
  Serial.println(UpTime);
  Serial.print("Down Time (ms): ");
  Serial.println(DownTime);
  Serial.print("Ratio: ");
  Serial.println(UpDownRatio);
}

void loop() {

  unsigned long currentMillis = millis();
  Blynk.run();

  /***************************** DOWN button **********************************/

  if (digitalRead(DownButtonPin) == HIGH) {

    if (DownButtonActive == false) { //Button pressed

      DownButtonActive = true;
      buttonTimer = millis();
    }

    if ((millis() - buttonTimer > longPressTime) && (DownLongPressActive == false)) { // Button Held

      DownLongPressActive = true;
      LED1State = !LED1State;
      digitalWrite(LED1, LED1State);
      while (digitalRead(DownButtonPin) == HIGH) { //Move motor while button pressed
        Serial.println("Holding DOWN");
        move_motors(0); // Move motor down
        digitalWrite(LED2, HIGH); //turn on red led while moving
      }
    }
  } else {

    if (DownButtonActive == true) {

      if (DownLongPressActive == true) { // Button Released following long hold
        NewDownTime = millis() - buttonTimer;
        DownLongPressActive = false;
        Serial.println("OFF");
        move_motors(512); // Stop moving motors
        digitalWrite(LED2, HIGH); //turn on red led while waiting for calibration
        // Wait for 3 seconds, to check if we were just moving or recalibrating
        delay(3000);
        digitalWrite(LED2, LOW); //calibration ended turn off red led
        // If button held blink led twice to confirm recalibration
        if (digitalRead(DownButtonPin) == HIGH) {
          //Blink red LED 2 times to confirm
          blink_red_led(100, 2);
          // Recalibration Instucted, write new down time to EEPROM
          DownTime = NewDownTime;
          EEPROM.write(30, DownTime);
          Serial.println("Down Time Saved as:");
          Serial.println(DownTime);

          // Set blinds as closed and update EEPROM
          CurrentState = 0; // blinds now closed
          Serial.println("Closed");
          EEPROM.write(0, CurrentState);
          delay(3000); //Wait 3 seconds to give user a chance to let go of button
          digitalWrite(LED1, LOW); //turn off the green led
        }
        //else just turn off led
        else {
          LED1State = !LED1State;
          digitalWrite(LED1, LED1State);
        }
      } else { // Button released following single press

        // Check if blinds are already closed
        if (CurrentState == 0) // blinds already closed
        {
          //Do nothing
          Serial.println("Blinds Already Closed!");
          blink_red_led(100, 2);
        }
        else {
          //Close the blinds
          //Flash the LED
          digitalWrite(LED1, HIGH);
          delay(100);
          digitalWrite(LED1, LOW);
          delay(100);
          digitalWrite(LED1, HIGH);
          digitalWrite(LED1, LOW);
          Serial.println("Closing...");

          currentMillis = millis();
          previousMillis = currentMillis;

          while (currentMillis - previousMillis <= DownTime) {
            move_motors(0); // move motor down
            digitalWrite(LED2, HIGH); // turn on red led
            currentMillis = millis();

            //Check if we pressed the button again
            if (digitalRead(DownButtonPin) == HIGH) {
              //Stop moving
              move_motors(512);
              // exit the loop
              break;
            }
          }
          //Flash LED
          digitalWrite(LED1, HIGH);
          delay(100);
          digitalWrite(LED1, LOW);
          delay(100);
          digitalWrite(LED1, HIGH);
          delay(100);
          digitalWrite(LED1, LOW);
          // Set blinds to closed
          CurrentState = 0; // blinds now closed
          EEPROM.write(0, CurrentState);
          Serial.println("Blinds Now Closed");
          digitalWrite(LED1, LOW); //turn off the green led

          //Stop moving motors
          move_motors(512);
        }
      }
      DownButtonActive = false;
      digitalWrite(LED2, LOW); // turn off red LED
    }
  }
  /***************************** END DOWN button *******************************/

  /******************************** UP button **********************************/
  if (digitalRead(UpButtonPin) == HIGH) {

    if (UpButtonActive == false) { //Button pressed

      UpButtonActive = true;
      buttonTimer = millis();
    }

    if ((millis() - buttonTimer > longPressTime) && (UpLongPressActive == false)) { // Button Held

      UpLongPressActive = true;

      digitalWrite(LED2, HIGH); //Turn on the red LED while moving
      while (digitalRead(UpButtonPin) == HIGH) { //Move motor while button pressed
        Serial.println("Holding UP");
        move_motors(1024); // Move motor up
      }
    }
  } else {

    if (UpButtonActive == true) {

      if (UpLongPressActive == true) { // Button Released following long hold
        digitalWrite(LED2, HIGH);
        NewUpTime = millis() - buttonTimer;
        UpLongPressActive = false;
        Serial.println("OFF");
        move_motors(512); // Stop moving motor
        // Wait for 3 seconds, to check if we were just moving or recalibrating
        delay(3000);
        digitalWrite(LED2, LOW);
        // If button held blink led twice to confirm recalibration
        if (digitalRead(UpButtonPin) == HIGH) {
          digitalWrite(LED1, LOW);
          delay(100);
          digitalWrite(LED1, HIGH);
          delay(100);
          digitalWrite(LED1, LOW);
          delay(100);
          digitalWrite(LED1, HIGH);
          delay(100);
          digitalWrite(LED1, LOW);
          // Recalibration Instructed, write new up time to EEPROM
          UpTime = NewUpTime;
          EEPROM.write(4, UpTime);
          Serial.println("Up Time Saved as:");
          Serial.println(UpTime);

          // Set blinds as open and update EEPROM
          CurrentState = 1; // blinds now open
          Serial.println("Open");
          EEPROM.write(0, CurrentState);
          delay(3000); //Wait 3 seconds to give user a chance to release the button
          digitalWrite(LED2, LOW);
        }
      } else { // Button released following single press

        // Check if blinds are already open
        if (CurrentState == 1) // blinds already open
        {
          //Do nothing
          Serial.println("Blinds Already Open!");
          blink_red_led(100, 2);
        }
        else {
          //Open the blinds
          //Flash the LED
          Serial.println("Opening...");
          // Begin Opening

          currentMillis = millis();
          previousMillis = currentMillis;

          while (currentMillis - previousMillis <= UpTime) {
            move_motors(1024); // move motor up
            digitalWrite(LED2, HIGH); // turn on red led while operating
            currentMillis = millis();

            //Check if button is pressed again
            if (digitalRead(UpButtonPin) == HIGH) {
              //Stop moving
              move_motors(512);
              // exit the loop
              break;
            }
          }
          // Set blinds to open
          CurrentState = 1; // blinds now open
          EEPROM.write(0, CurrentState);
          Serial.println("Blinds Now Open");
          digitalWrite(LED1, HIGH); //turn on the green led
          //Stop moving motor
          move_motors(512);
          digitalWrite(LED2, LOW); //turn off the red led
        }
      }
      UpButtonActive = false;
    }
  }
  digitalWrite(LED2, LOW); // turn off the red led
  /****************************** END UP button ********************************/

  /********************** Google Assistant Control *****************************/
  //Read from our subscription queue until we run out, or
  //wait up to 20 seconds for subscription to update
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Apri) {
      buttonTimer = millis();
      if (!strcmp((char*) Apri.lastread, "Apri")) {
        currentMillis = millis();
        previousMillis = currentMillis;

        while (currentMillis - previousMillis <= UpTime) {
          move_motors(1024); // move motor up
          digitalWrite(LED2, HIGH); // turn on red led while operating
          currentMillis = millis();
        }
        // Set blinds to open
        CurrentState = 1; // blinds now open
        EEPROM.write(0, CurrentState);
        Serial.println("Blinds Now Open");
        digitalWrite(LED1, HIGH); //turn on the green led
        //Stop moving motor
        move_motors(512);
        digitalWrite(LED2, LOW); //turn off the red
      }
    }
    if (subscription == &Chiudi) {
      buttonTimer = millis();
      if (!strcmp((char*) Chiudi.lastread, "Chiudi")) {
        currentMillis = millis();
        previousMillis = currentMillis;

        while (currentMillis - previousMillis <= DownTime) {
          move_motors(0); // move motor down
          digitalWrite(LED2, HIGH); // turn on red led
          currentMillis = millis();
        }
        //Flash LED
        digitalWrite(LED1, HIGH);
        delay(100);
        digitalWrite(LED1, LOW);
        delay(100);
        digitalWrite(LED1, HIGH);
        delay(100);
        digitalWrite(LED1, LOW);
        // Set blinds to closed
        CurrentState = 0; // blinds now closed
        EEPROM.write(0, CurrentState);
        Serial.println("Blinds Now Closed");
        digitalWrite(LED1, LOW); //turn off the green led

        //Stop moving motors
        move_motors(512);
      }
    }
  }
  /********************* END Google Assistant Control **************************/
}
//END Void Loop

/****************************** Motor control ********************************/
//Note: ESP8266 MAX Resolution 1024 - ARDUINO MAX Resolution 255
void move_motors(int potValue)
{
  if (potValue < 512) {
    potValue = 1024;
    int mappedVal = map(potValue - 512, 0, 512, 0, 1024);
    digitalWrite(E1, LOW);
    analogWrite(M1, mappedVal);   //PWM Speed Control
    delay(30);
  } else {
    //Going backward
    int mappedVal = map(potValue - 512, 0, 512, 0, 1024);
    digitalWrite(M1, LOW);
    analogWrite(E1, mappedVal);   //PWM Speed Control
    delay(30);
  }
}
/**************************** END Motor control ******************************/

/******************************** LED Blink **********************************/
void blink_red_led(int timeDelay, int noTimes) {
  noTimes = noTimes * 2;
  for (int i = 1; i <= noTimes; i++)
  {
    LED2State = !LED2State;
    digitalWrite(LED2, LED2State);
    delay(timeDelay);
  }
}

void blink_green_led(int timeDelay, int noTimes) {
  for (int i = 1; i <= noTimes; i++)
  {
    LED1State = !LED1State;
    digitalWrite(LED1, LED1State);
    delay(timeDelay);
  }
}
/****************************** END LED Blink ********************************/

/******************************** Wifi Code **********************************/
// connect to wifi – returns true if successful or false if not
boolean connectWifi() {
  boolean state = true;
  int i = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WiFi_SSID, WiFi_PASS);
  Serial.println("");
  Serial.println("Connecting to WiFi");

  // Wait for connection
  Serial.print("Connecting ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 10) {
      state = false;
      break;
    }
    i++;
  }

  if (state) {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(WiFi_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("");
    Serial.println("Connection failed.");
  }

  return state;
}
/****************************** END Wifi Code ********************************/

/********************************* MQTT Code *********************************/
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
/****************************** END MQTT Code ********************************/
