# FlexSensorControllerWithIMU

## 1.0 Introduction

This is the code for my final year project at the University of Manchester. In this project, a controller is used with flex sensors and an IMU to replace a traditional keyboard and mouse for disabled users. This controller uses the Arduino IDE with its libraries, and the `blekeyboard.h` and `bleMouse.h` libraries, which you can find in the installation section. The goal of this controller is to send keyboard strokes using the flex sensors, and use the IMU to set different wrist orientations that send unique keyboard strokes. For example, if the user's palm is facing up and they flex their thumb, index, middle, third, and pinky fingers, it would send "A", "B", "C", and "D". Now, if the user rotates their palm 180 degrees and flexes their thumb, index, middle, third, and pinky fingers, it would send "W", "Y", "V", "M", and "L".

There is also a mouse mode within the controller. The idea is that the user can press a button to switch between keyboard and mouse modes. The mouse mode works with the flexed fingers acting as the left and right click buttons, and the gyroscope acting as the way to move the mouse.

## 2.0 Component Breakdown

- [ESP32](https://www.adafruit.com/product/5400)
- [IMU](https://thepihut.com/products/fermion-bmx160-9-axis-sensor?srsltid=AfmBOooHgW4BE-4IyZJ4Hw-fLh-15MH_mMO_FT3TeNjS9t5ynJq3zvxx)
- [Flex Sensors](https://thepihut.com/products/short-flex-sensor)
- [Amplifier for Flex Sensors](https://www.mouser.co.uk/ProductDetail/Texas-Instruments/LM324N?qs=VolsR0DjNPqtt3qB38bTqw%3D%3D&srsltid=AfmBOorFGjzcyBDvUDmJaR8VD3igG85v7neHHGXI1ksrMev2mdYswIft)
- [3.7 Li-Po Battery](https://www.kiwi-electronics.com/en/lithium-polymer-li-po-battery-3-7v-1200mah-1045?country&srsltid=AfmBOooEjzT6RcRR4mf8AO5ckx_v4nLIEa5GYKjxjeKD5g3jgqPOplwaVfQ&gQT=1)

## 3.0 Context on How It Works

As seen in the image below, it shows how the keyboard code works in an abstract view. Note that the mouse code works in a similar structure, but due to the free software I was using (e.g., Lucid), I couldn't add more block diagrams.

The code works by first looping an LED on and off until it is connected to a device via Bluetooth. From here, the controller will be able to send keyboard strokes when the flex sensor is bent. This works by a series of if and switch statements.

Note that the mouse code is in a separate document at this time due to some issues discussed in Section 6.

## 4.0 How to Install

1. You will need the Arduino IDE, which can be found here: [Arduino IDE](https://www.arduino.cc/en/software/)
   
2. If you haven't done so already, you need to install the ESP32 into the Arduino boards. To do this, go to **Tools > Board > Boards Manager** and search for "ESP32". Click "Install" for the "esp32" package by Espressif Systems.

   Or, go to:
   - **On Windows**: Go to **File > Preferences**
   - **On macOS**: Go to **Arduino > Preferences**
   
   In the **Additional Boards Manager URLs** field, type:
    https://dl.espressif.com/dl/package_esp32_index.json


3. Install the extra BLE libraries. You can download the zip files for these libraries here: 
- [Bluetooth Keyboard](https://github.com/T-vK/ESP32-BLE-Keyboard) 
- [Bluetooth Mouse](https://github.com/T-vK/ESP32-BLE-Mouse)
- [IMU](https://github.com/DFRobot/DFRobot_BMX160)

4. You will need to add these ZIP files to the Arduino IDE:
- Open the Arduino IDE.
- Go to **Sketch > Include Library > Add .ZIP Library**.
- Find the ZIP file you downloaded and click **Open**.

## 5.0 Technical Details

For the flex sensors circuit, a voltage divider with the flex sensor was used, where the midpoint was fed into a voltage follower circuit (with the package LM324N). R1 was the flex sensors and R2 was a 36kΩ resistor, as seen in the data sheet for the flex sensor. You can find the data sheet [here](https://spectra-symbol-landing.s3.us-west-1.amazonaws.com/data-sheets/Flex-Sensor-Datasheet-v2019a.pdf). Within this link, you can find the formula used to set threshold voltages for logic transitions within the code.

## 6.0 Issues and Further Improvements

The keyboard Bluetooth and mouse Bluetooth cannot be used interchangeably in the same program. My current workaround, which needs more testing, is using flash memory on the ESP32 and then restarting it in the correct Bluetooth mode. However, this is annoying as it causes the users to constantly reconnect.

A good way to get around this would be to combine both libraries into a single API.

Additional work needs to be done to make this controller into a fully working keyboard by adding more inputs. Using a binary approach to the flex patterns would lead to:

**No inputs = 2^(No. of flex sensors) * No. of IMU Orientations**

This, combined with a word algorithm or attaching certain words to inputs, would make this more usable as a keyboard.

