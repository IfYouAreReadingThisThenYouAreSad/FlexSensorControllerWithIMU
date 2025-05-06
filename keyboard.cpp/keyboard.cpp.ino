//final project code


// eps32 starts in pin 16 of bread board
// angolgoue is in pin 22 of bread board // pin 33 of esp
 
// anglogue in pin 21 of bread board // pin 32 of esp
// angluge pin 20 of bread board       // pin 35 of esp
 
#include <BleKeyboard.h>
#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <BleMouse.h>
 
BleKeyboard bleKeyboard;
BleMouse bleMouse;
 
#define SDA_PIN 22
#define SCL_PIN 20
 
#define ANALOG_PIN_0 25
#define ANALOG_PIN_1 26
#define ANALOG_PIN_2 34
#define ANALOG_PIN_3 39
 
#define LED_Blue 33
#define LED_Green 27
#define LED_Red 12
#define BUTTON_PIN 13
 
volatile bool mode = false;  // false = Keyboard mode, true = Mouse mode
volatile unsigned long lastInterruptTime = 0;
 
void IRAM_ATTR toggle_mode() { 
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200) {  // 200ms debounce
        mode = !mode;  // Toggle mode
        Serial.println("Changed States");
        digitalWrite(LED_Red, mode);
    }
    lastInterruptTime = interruptTime;
}
 
DFRobot_BMX160 bmx160;
sBmx160SensorData_t magData, gyroData, accelData;
 
unsigned long currentMillis, previousMillis = 0;
volatile float gyroX = 0, gyroY = 0, gyroZ = 0, gyroX_angle = 0, gyroY_angle = 0, gyroZ_angle = 0;
 
void setup() {
    Serial.begin(115200);
 
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
 
    // Configure LED pin
    pinMode(LED_Blue, OUTPUT);
    pinMode(LED_Green, OUTPUT);
    pinMode(LED_Red, OUTPUT);
 
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggle_mode, RISING);
   
    bleKeyboard.begin();
 
    if (bmx160.begin() != 0) {
        Serial.println("ERROR: BMX160 NOT DETECTED!");
    } else {
        Serial.println("BMX160 Initialized Successfully!");
    }
}
 
void loop() {
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_Blue, LOW);
    digitalWrite(LED_Red, LOW);
    while (!bleKeyboard.isConnected()) {
    Serial.println("Waiting for BLE connection...");
    digitalWrite(LED_Blue, HIGH);
    digitalWrite(LED_Red, HIGH);
    delay(500);
    digitalWrite(LED_Blue, LOW);
    digitalWrite(LED_Red, LOW);
    delay(500);
    }
 
    while (bleKeyboard.isConnected()) {   
        //currentMillis = millis();
        int rawValue_Thumb = analogRead(ANALOG_PIN_0), rawValue_Index = analogRead(ANALOG_PIN_1), rawValue_Middle =analogRead(ANALOG_PIN_2), rawValue_Third = analogRead(ANALOG_PIN_3);
        float voltage_Thumb = (rawValue_Thumb/ 4095.0) * 3.3, voltage_Index = (rawValue_Index / 4095.0) * 3.3, voltage_Middle = (rawValue_Middle / 4095.0) * 3.3, voltage_Third = (rawValue_Third / 4095.0) * 3.3;  // Convert to voltage
 
        // ---- BMX160 SENSOR SECTION ---- //
 
            // Mouse clicking code (Right Click)
                if (voltage_Index < 1.3) {
                Serial.println("Index activated");
                bleKeyboard.write(KEY_LEFT_ARROW);
                digitalWrite(LED_Red, HIGH);
               
              }
              else{
                bleKeyboard.release(KEY_LEFT_ARROW);
                digitalWrite(LED_Red, LOW);
              }
 

            // Mouse clicking code (Left Click)
           
                if (voltage_Middle < 1.3) {
                Serial.println("Middle Activated");
                bleKeyboard.write(KEY_UP_ARROW);
                digitalWrite(LED_Blue, HIGH);
                
              }
              else{
                bleKeyboard.release(KEY_UP_ARROW);
                digitalWrite(LED_Blue, LOW);
              }
 
            // Mouse clicking code (Scrool)
 
                if (voltage_Third < 1.3) {
                Serial.println("Third Activated");
                bleKeyboard.write(KEY_RIGHT_ARROW);
                digitalWrite(LED_Green, HIGH);
               
              }
              else{
                bleKeyboard.release(KEY_RIGHT_ARROW);
                digitalWrite(LED_Green, LOW);
              }             



              delay(500);
 
            /* debugging code
           // Serial.print("GyroX Angle1: ");
       
            //Serial.println(gyroX_angle);
              Serial.print("Angle X: ");
              Serial.print(gyroX_angle);
              Serial.print("| Y: ");
              Serial.print(gyroY_angle);
              Serial.print("| Z: ");
              Serial.print(gyroZ_angle);
              Serial.println(" m/s²");
 
              Serial.print("Acceleration X: ");
              Serial.print(accelData.x / 100.0);
              Serial.print(" m/s² | Y: ");
              Serial.print(accelData.y / 100.0);
              Serial.print(" m/s² | Z: ");
              Serial.print(accelData.z / 100.0);
              Serial.println(" m/s²");



              delay(80); */
 
        }
 
    }
 
    // Reset LEDs when BLE is disconnected
 

 