//Keyboard code for protyope v4.6


#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <BleKeyboard.h>
BleKeyboard bleKeyboard;
bool connected = false;
#define SDA_PIN 21        // I2C SDA (BMX160)
#define SCL_PIN 22        // I2C SCL (BMX160)
#define ANALOG_PIN_1 32   // Flex Sensor on D32 index fingure
#define ANALOG_PIN_2 33  // flex sensor on d32 minddle figure
#define ANALOG_PIN_3 35 // flex sebsir on D35 third figure or something else
#define LED_PIN 2         // Built-in LED on ESP32 Dev Board
#define LED_Yellow 23
#define LED_Green 19
#define LED_White 18
#define LED_Red 4
#define BUTTON_PIN 15     // Button on D15
DFRobot_BMX160 bmx160;  // BMX160 object
sBmx160SensorData_t magData, gyroData, accelData;  // Separate structs for sensor data
 
unsigned long currentMillis, previousMillis = 0;  // Timing variables
float gyroX = 0, gyroX_angle = 0; // Gyroscope variables
 
enum ArmRotationState {RightRotate, LeftRotate, UpRight};
ArmRotationState CurrentArmState = UpRight;
 
void setup() {
    Serial.begin(115200);
 
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
 
    // Configure LED pin
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_Green,OUTPUT);
    pinMode(LED_Yellow,OUTPUT);
    pinMode(LED_White,OUTPUT);
 
    bleKeyboard.begin();
    if (bmx160.begin() != 0) {
      Serial.println("ERROR: BMX160 NOT DETECTED!");
    }
    else {Serial.println("BMX160 Initialized Successfully!");}
}
 
void loop() {
 
    while (!bleKeyboard.isConnected()) {
        Serial.println("Waiting for BLE connection...");
        digitalWrite(LED_PIN, HIGH);
        delay(500);  // LED ON for 500ms
        digitalWrite(LED_PIN, LOW);
        delay(500);  // LED OFF for 500ms
    }
  while(bleKeyboard.isConnected()){
    currentMillis = millis(); // Update currentMillis for timing
 
    // ---- FLEX SENSOR SECTION ---- //
int rawValue_Second = analogRead(ANALOG_PIN_1), rawValue_Third =analogRead(ANALOG_PIN_2), rawValue_First = analogRead(ANALOG_PIN_3);
        float voltageFirst = (rawValue_First / 4095.0) * 3.3, voltageSecond = (rawValue_Second / 4095.0) * 3.3, voltageThird = (rawValue_Third / 4095.0) * 3.3;  // Convert to voltage
   
 
    /*
    Serial.print("Flex Sensor Voltage: ");
    Serial.print(voltage);
 
 
    // If voltage drops below 1.6V, turn LED on
    if (voltage < 1.6) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println(" -> BENT! LED ON");
    } else
        digitalWrite(LED_PIN, LOW);
        Serial.println(" -> Straight. LED OFF");
      */
   
    
 
    // ---- BMX160 SENSOR SECTION ---- //
  if (currentMillis - previousMillis >= 100) {  // Sample every 100ms
    previousMillis = currentMillis;
 
    sBmx160SensorData_t magData, gyroData, accelData;
    bmx160.getAllData(&magData, &gyroData, &accelData);
 
    // Get gyro X-axis angular velocity (deg/s)
    gyroX = gyroData.x;
 
    // Compute the angle change over time
    float deltaAngle = gyroX * 0.1;  // 100ms assumed time interval (0.1s)
    gyroX_angle += deltaAngle;  // Update overall angle
 
    // Reset angle to zero when accelerometer is upright, with a tolerance range
    float zAccel = accelData.z / 100.0;  // Normalized z-axis value
    if (zAccel > 0.09 && zAccel < 0.11) {  // Allow a small tolerance range around 0.1
      gyroX_angle = 0.00;  // Reset to 0 if upright (within tolerance range)
      CurrentArmState = UpRight;
    }
    else if(gyroX_angle >= 1.6){
      CurrentArmState = LeftRotate;
    }
    else if(gyroX_angle <= -1.6){
      CurrentArmState = RightRotate;
    }
    else{
      CurrentArmState = UpRight;
    }
 
    // Print Gyroscope angle
    /*
    Serial.print("Gyro X Position (Angle): ");
    Serial.print(gyroX_angle);
    Serial.println(" degrees");
 
    Serial.print("Acceleration X: ");
    Serial.print(accelData.x / 100.0);
    Serial.print(" m/s² | Y: ");
    Serial.print(accelData.y / 100.0);
    Serial.print(" m/s² | Z: ");
    Serial.print(accelData.z / 100.0);
    Serial.println(" m/s²");
    */
 
    switch(CurrentArmState){
 
    case(UpRight):
    digitalWrite(LED_Yellow, HIGH);
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_White, LOW);
    Serial.printf("State: UpRight");
    if (voltageFirst < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('a');
    } else{
        bleKeyboard.release('a');
        digitalWrite(LED_PIN, LOW);
    }
    if (voltageSecond < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('w');
    } else{
        bleKeyboard.release('w');
        digitalWrite(LED_PIN, LOW);
    }
    if (voltageThird < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('d');
    } else{
        bleKeyboard.release('d');
        digitalWrite(LED_PIN, LOW);
    }
 
    break;
 
    case(RightRotate):
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Green, HIGH);
    digitalWrite(LED_White, LOW);
    Serial.printf("State: RightRotate");
    if (voltageFirst < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('t');
    } else{
        bleKeyboard.release('t');
        digitalWrite(LED_PIN, LOW);
    }
    if (voltageSecond < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('h');
    } else{
        bleKeyboard.release('h');
        digitalWrite(LED_PIN, LOW);
    }
     if (voltageThird < 1.4) {
       digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('e');
    } else{
        bleKeyboard.release('e');
        digitalWrite(LED_PIN, LOW);
    }
    break;
 
    case(LeftRotate):
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_White, HIGH);
    Serial.printf("State: LeftRotate");
    if (voltageFirst < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('u');
    } else{
        bleKeyboard.release('u');
        digitalWrite(LED_PIN, LOW);
    }
    if (voltageSecond < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('r');
    } else{
        bleKeyboard.release('r');
        digitalWrite(LED_PIN, LOW);
    }
    if (voltageThird < 1.4) {
        digitalWrite(LED_PIN, HIGH);
       bleKeyboard.press('n');
    } else{
        bleKeyboard.release('n');
        digitalWrite(LED_PIN, LOW);
    }
    break;
 
    }
  }
 
  delay(10);  // Slow down loop for stability
 
  }
 
}