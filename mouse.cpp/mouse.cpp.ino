#include <BleKeyboard.h>
#include <BleMouse.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>

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

LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C address 0x6A

unsigned long currentMillis, previousMillis = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float gyroX_angle = 0, gyroY_angle = 0, gyroZ_angle = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(LED_Blue, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Red, OUTPUT);

  bleMouse.begin();

  if (myIMU.begin() != 0) {
    Serial.println("ERROR: LSM6DS3 NOT DETECTED!");
  } else {
    Serial.println("LSM6DS3 Initialized Successfully!");
  }
}

void loop() {
  if (!bleMouse.isConnected()) {
    Serial.println("Waiting for BLE connection...");
    digitalWrite(LED_Blue, HIGH);
    digitalWrite(LED_Red, HIGH);
    delay(500);
    digitalWrite(LED_Blue, LOW);
    digitalWrite(LED_Red, LOW);
    delay(500);
    return;
  }

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;

    // Read raw gyro values
    gyroX = myIMU.readFloatGyroX();
    gyroY = myIMU.readFloatGyroY();
    gyroZ = myIMU.readFloatGyroZ();

    // Read acceleration
    float accelX = myIMU.readFloatAccelX();
    float accelY = myIMU.readFloatAccelY();
    float accelZ = myIMU.readFloatAccelZ();

    // Update angle based on gyro
    gyroX_angle += (gyroX * 0.01); // scaled by 10ms
    gyroY_angle += (gyroY * 0.01);
    gyroZ_angle += (gyroZ * 0.01);

    // Auto-reset to neutral if upright
    if (fabs(accelY - (-0.01)) < 0.02) gyroX_angle = 0;
    if (fabs(accelX) < 0.02) gyroY_angle = 0;

    // Mouse movement X axis
    if (gyroX_angle > 0.00) {
      bleMouse.move(-gyroX_angle, 0);
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Red, HIGH);
      digitalWrite(LED_Blue, LOW);
    } else if (gyroX_angle < 0.00) {
      bleMouse.move(-gyroX_angle, 0);
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Red, LOW);
      digitalWrite(LED_Blue, HIGH);
    }

    // Mouse movement Y axis
    if (gyroY_angle > 0.00) {
      bleMouse.move(0, gyroY_angle);
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Red, HIGH);
      digitalWrite(LED_Blue, HIGH);
    } else if (gyroY_angle < 0.00) {
      bleMouse.move(0, gyroY_angle);
      digitalWrite(LED_Green, LOW);
      digitalWrite(LED_Red, HIGH);
      digitalWrite(LED_Blue, LOW);
    }


// Mouse clicking code (Right Click)
            /*
                if (voltage_Index < 1.6) {
                bleMouse.click(MOUSE_LEFT);
                digitalWrite(LED_Blue, HIGH);
                
              } 
              else{
                bleMouse.release(MOUSE_LEFT);
                digitalWrite(LED_Blue, LOW);
              }
              */

            // Mouse clicking code (Left Click)
                /*
                if (voltage_Middle < 1.6) {
                bleMouse.click(MOUSE_RIGHT);
                digitalWrite(LED_Blue, HIGH);
                
              } 
              else{
                bleMouse.release(MOUSE_RIGHT);
                digitalWrite(LED_Blue, LOW);
              }
                */

            // Mouse clicking code (Scrool)
              /*
                if (voltage_Third < 1.3 && voltage_Third >= 1.1) {
                bleMouse.move(0,0,1);; //scrolling up
                digitalWrite(LED_PIN, HIGH);
                
              } 
              else if ( voltage_Third < 1.1){
                bleMouse.move(0,0,-1);
                digitalWrite(LED_PIN, LOW);
              }              
                */


    // Debugging output
    Serial.print("Gyro Angles - X: ");
    Serial.print(gyroX_angle);
    Serial.print(" | Y: ");
    Serial.print(gyroY_angle);
    Serial.print(" | Z: ");
    Serial.println(gyroZ_angle);

    Serial.print("Accel - X: ");
    Serial.print(accelX);
    Serial.print(" | Y: ");
    Serial.print(accelY);
    Serial.print(" | Z: ");
    Serial.println(accelZ);

    delay(80); // debounce delay
  }

  delay(10);
  digitalWrite(LED_Green, LOW);
  digitalWrite(LED_Blue, LOW);
  digitalWrite(LED_Red, LOW);
}
