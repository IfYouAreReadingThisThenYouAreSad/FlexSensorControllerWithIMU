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
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_Green, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_White, OUTPUT);
    pinMode(LED_Red, OUTPUT);

    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggle_mode, RISING);
    
    bleMouse.begin();

    if (bmx160.begin() != 0) {
        Serial.println("ERROR: BMX160 NOT DETECTED!");
    } else {
        Serial.println("BMX160 Initialized Successfully!");
    }
}

void loop() {
    while (!bleMouse.isConnected()) {
        Serial.println("Waiting for BLE connection...");
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }

    while (bleMouse.isConnected()) {    
        currentMillis = millis();
        int rawValue_Index = analogRead(ANALOG_PIN_1), rawValue_Middle =analogRead(ANALOG_PIN_2), rawValue_Third = analogRead(ANALOG_PIN_3);
        float voltage_Index = (rawValue_Index / 4095.0) * 3.3, voltage_Middle = (rawValue_Middle / 4095.0) * 3.3, voltage_Third = (rawValue_Third / 4095.0) * 3.3;  // Convert to voltage

        // ---- BMX160 SENSOR SECTION ---- //
        if (currentMillis - previousMillis >= 10) {  
            previousMillis = currentMillis;

            bmx160.getAllData(&magData, &gyroData, &accelData);

            // Get gyro X-axis angular velocity (deg/s)
            gyroX = gyroData.x;
            gyroY = gyroData.y;
            gyroZ = gyroData.z;

            // Compute the angle change over time
            gyroX_angle += (gyroX * 0.1);  // Update overall angle
            gyroY_angle += (gyroY * 0.1);  // Update overall angle
            gyroZ_angle += (gyroZ * 0.1);  // Update overall angle

            // Implement decay effect (gradually return to zero)
            //gyroX_angle *= 0.95;

            // Ensure gyroX_angle stays within -10 to 10
            //gyroX_angle = constrain(gyroX_angle, -10, 10);

            // Map gyroX_angle for smoother movements
            //gyroX_angle = map(gyroX_angle, -10, 10, -5, 5);

            // Reset angle to zero when accelerometer is upright



            if (fabs((accelData.y / 100.0) - (-0.01)) < 0.02) {  // Small tolerance range
                gyroX_angle = 0.00;
                //gyroZ_angle = 0.00;
            }

            if (fabs(accelData.x / 100.0) < 0.02) {  // Small tolerance range
                gyroY_angle = 0.00;
           
            }





            // Movement logic x direction
            if (gyroX_angle > 0.00) {  // Moving left
                bleMouse.move(-gyroX_angle, 0);
                digitalWrite(LED_Yellow, HIGH);
                digitalWrite(LED_Green, LOW);
                digitalWrite(LED_Red, LOW);
                digitalWrite(LED_White, LOW);
                digitalWrite(LED_PIN, LOW);
            } 
            else if (gyroX_angle < 0.00) {  // Moving right
                bleMouse.move(-gyroX_angle, 0);
                digitalWrite(LED_Yellow, LOW);
                digitalWrite(LED_Green, HIGH);
                digitalWrite(LED_Red, LOW);
                digitalWrite(LED_White, LOW);
                digitalWrite(LED_PIN, LOW);
            }


              // Movement logic y direction
            if (gyroY_angle > 0.00) {  // Moving down
                bleMouse.move(0, gyroY_angle);
                digitalWrite(LED_Yellow, LOW);
                digitalWrite(LED_Green, LOW);
                digitalWrite(LED_Red, LOW);
                digitalWrite(LED_White, HIGH);
                digitalWrite(LED_PIN, LOW);
            } 
            else if (gyroY_angle < 0.00) {  // Moving up
                bleMouse.move(0, gyroY_angle);
                digitalWrite(LED_Yellow, LOW);
                digitalWrite(LED_Green, LOW);
                digitalWrite(LED_Red, HIGH);
                digitalWrite(LED_White, LOW);
                digitalWrite(LED_PIN, LOW);
            }


            // Mouse clicking code (Right Click)
                if (voltage_Index < 1.6) {
                bleMouse.click(MOUSE_LEFT);
                digitalWrite(LED_PIN, HIGH);
                
              } 
              else{
                bleMouse.release(MOUSE_LEFT);
                digitalWrite(LED_PIN, LOW);
              }


            // Mouse clicking code (Left Click)
            
                if (voltage_Middle < 1.6) {
                bleMouse.click(MOUSE_RIGHT);
                digitalWrite(LED_PIN, HIGH);
                
              } 
              else{
                bleMouse.release(MOUSE_RIGHT);
                digitalWrite(LED_PIN, LOW);
              }

            // Mouse clicking code (Scrool)

                if (voltage_Third < 1.3 && voltage_Third >= 1.1) {
                bleMouse.move(0,0,1);; //scrolling up
                digitalWrite(LED_PIN, HIGH);
                
              } 
              else if ( voltage_Third < 1.1){
                bleMouse.move(0,0,-1);
                digitalWrite(LED_PIN, LOW);
              }              





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

        delay(10);
    }

    // Reset LEDs when BLE is disconnected
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_White, LOW);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_Red, LOW);
}
