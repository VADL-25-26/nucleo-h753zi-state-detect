# State Detection Logic Process

Project Files Associated with the state detection and VN-100 IMU integration onto STM32F446RE and STM32H753ZI Nucleo Boards. 

Configuration and Setup Process:

1. IMU + State Detection Main Loop Testing

Clone from "patrick-imu-branch"

Follow the connections in the image below when using the STM32 F446RE:
<img width="4510" height="2615" alt="F446RE IMU Testbench Pin Diagram drawio" src="https://github.com/user-attachments/assets/5e5ceae0-ebc4-4065-9fcf-2725f0e73dfe" />

Ensure that TIM7, USART1 & USART2 are enabled.
USART1 and USART2 must have a Baud of 115200 and WL of 8 (including parity).

Run the main.c file in either debug or run modes
Launch a Serial Monitor at 115200 and read from COM connected to STM32. 
