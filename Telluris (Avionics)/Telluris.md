<img width="960" height="840" alt="image" src="https://github.com/user-attachments/assets/404ac74b-fb4c-4355-8b08-122d665ab8e7" />

[Telluris Code](https://app.notion.com/p/Telluris-Wiki-22edc7fa243a802baae4f8249a0813ee#23edc7fa243a802d893bfaf9453b8a5b)

[Telluris Website](https://gauchorocketprojec.wixsite.com/gaucho-rocket-projec/projects)

### Thrust Vector Controls

- Using a servo-based thrust vector control, we are able to control the trajectory of our rocket to achieve precision landing.

### Deployable Landing Legs
- Our landing legs, coupled with a suspension system, deploy during our descent as the rocket attempts its landing.

### Flight Computer
The flight computer serves as the central controller of the rocket, managing critical functions from liftoff and thrust vector control to the deployment of the landing legs.
<img width="1898" height="1504" alt="image" src="https://github.com/user-attachments/assets/0a34cea5-6fb2-4d90-9ae3-136a6b6f7d7f" />

### Schematic 

![Schematic](https://github.com/user-attachments/assets/9a484144-c24b-4605-bac4-c39f88615eeb)

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
