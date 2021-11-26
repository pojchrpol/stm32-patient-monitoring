# Patient Monitoring System

## Created on B-L475E-IOT01A board with STM32L475VGT6

In this project, we aim to build a system, COPEMON (COvid Patients Enhanced Monitoring), in which the health status of the COVID-19 patients, especially elderly, is monitored. Our system is able to monitor the temperature and breathing of patients, using temperature, humidity and pressure sensors, as well as detecting if the patient has fallen using accelerometer and any usual movement using the gyroscope and magnetometer. This is done by sending warnings when the readings of these sensors do not meet the criteria of the predetermined threshold values. 

This microcontroller used in this project is B-L475E-IOT01A, manufactured by STMicroelectronics. It has STM32L475VGT6 as the microprocessors, together with LED and on-board sensors, of which four are used in this project: HTS221 (Temperature sensor and Humidity sensor), LSM6DSL (Accelerometer and Gyroscope), LIS3MDL (Magnetometer), and LPS22HB (Pressure sensor). CHIPACU, the terminal program, is enabled by the Tera Term program, and DEBUG_CONSOLE in STM32CubeIDE is used in the programming process.
