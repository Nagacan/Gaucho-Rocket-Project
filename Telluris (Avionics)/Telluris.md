<img width="480" height="480" alt="image" src="https://github.com/user-attachments/assets/404ac74b-fb4c-4355-8b08-122d665ab8e7" />

![Telluris Code](https://app.notion.com/p/Telluris-Wiki-22edc7fa243a802baae4f8249a0813ee#23edc7fa243a802d893bfaf9453b8a5b)

![Telluris Website](https://gauchorocketprojec.wixsite.com/gaucho-rocket-projec/projects)

### Thrust Vector Controls

- Using a servo-based thrust vector control, we are able to control the trajectory of our rocket to achieve precision landing.

### Deployable Landing Legs
- Our landing legs, coupled with a suspension system, deploy during our descent as the rocket attempts its landing.

### Flight Computer
The flight computer serves as the central controller of the rocket, managing critical functions from liftoff and thrust vector control to the deployment of the landing legs.

<img width="480" height="480" alt="image" src="https://github.com/user-attachments/assets/0a34cea5-6fb2-4d90-9ae3-136a6b6f7d7f" />

### Schematic 

![Schematic](https://github.com/user-attachments/assets/9a484144-c24b-4605-bac4-c39f88615eeb)


>  *The ESP32 uses a USB-C connection for uploading code.*
---

##  Requirements

###  Arduino IDE Setup

- Make sure you're using the **Arduino IDE**
- Select the following under **Tools**:
  - **Board:** `ESP32 Dev Module`
  - **Port:** The correct COM port your ESP32 is connected to
  - **Baud Rate:** Check the correct baud rate for serial monitor and anywhere else relevant (115200 typically)

---

###  Libraries to Install

Install the following libraries via the **Arduino Library Manager**, or manually download them from GitHub and include them:

- **Madgwick** by Arduino  
- **Mahony** by Arduino  
- **Adafruit BMP3XX** by Adafruit  
- **Adafruit BusIO** by Adafruit  
- **Adafruit Unified Sensor** by Adafruit  
- **ESP32SPISlave** by hideakitai  
- **ICM20948_WE** by Wolfgang Ewald  
- **SparkFun BMP581 Arduino Library** by SparkFun Electronics  
- **SparkFun 9DoF IMU Breakout - ICM 20948** by SparkFun Electronics  

> 💡 **Windows users**: Install the [CP210x Universal Windows Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) if your board doesn’t appear under **Ports**.

---

##  IMU-20948 and DMP Setup (for IMU)

1. Navigate to:  
   `Documents/Arduino/libraries/SparkFun_ICM-20948_ArduinoLibrary/src/util/`

2. Open `ICM_20948_C.h` and uncomment the following line:   `#define ICM_20948_USE_DMP`
3. Wire the ICM-20948 to the ESP32 using this schematic: <p align="center"> <img width="480" height="480" alt="image" src="https://github.com/user-attachments/assets/55271a32-4c1b-4f5f-b4ea-8b9dc3966151" alt="IMU wiring schematic"/> </p>
4. Open or paste the provided sketch from this repository into a new Arduino IDE project.
5. Upload the sketch, and hold the boot button on the ESP32 while uploading
6. Open Serial Moniter at `115200 baud` and when prompted enter any key

## Key Features
| Sub-system | Parts/Notes |
|------------|-------------|
| MCU | Espressif **ESP32-S3** module, 240 MHz dual-core, Wi-Fi & Bluetooth LE|
| Inertial sensing | **ICM-20948** 9-axis IMU (gyro + accel + mag) |
| Altimetry | **BMP581** absolute barometer (32-bit FIFO, 5 Pa RMS) |
| Power | 2-stage regulation: **LD1117-5 V** & **SPX3819-3 V3** LDOs |
| Pyro outputs | Low-side MOSFETs (**DMN3023L-7**) for ejection charges |
| User feedback | JST-XH debug header |
| Connectors | **XT30UPB** main supply; screw terminals for igniters |
| Form factor | 4-layer, 50 × 80 mm PCB < 15 g bare |
