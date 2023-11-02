# README for Thermostat Control System

## Project Overview
This project, "Heater Simulator Embedded Systems," was designed to simulate the temperature regulation aspect of an heating system using embedded systems, specifically targeting the Texas Instruments development boards. The problem it addresses is real-time hardware interaction and control, common in many IoT devices today, like smart thermostats. By managing hardware peripherals and implementing control logic, the system maintains a user-defined temperature setpoint, mimicking a fundamental smart climate control scenario.

## Prerequisites
Embedded system hardware (like the Texas Instruments board)
C environment for development (like Code Composer Studio)
Basic knowledge of C, GPIO, I2C, UART, and Timer peripherals

## Installation
Clone the repository to your local machine. If you're using an IDE, open the project from the directory you've cloned it into. Ensure that the target hardware matches the one defined in the project settings.
``` 
git clone https://github.com/ColeBaxendale/AC_Simulator_Embedded_Systems.git
```

## Usage
The system reads temperature data from an I2C temperature sensor. If the current temperature is below the setpoint, the heater (an LED in this case) turns on. The user can increase or decrease the setpoint using buttons. The system prints status messages over UART.

**Here's a snippet of how the temperature reading is done:**
```
int16_t readTemp(void){
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)){
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        if (rxBuffer[0] & 0x80){
            temperature |= 0xF000;
        }
    }
    else{
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
    }
    return temperature;
}
```

**Button interrupts are handled as follows:**
```
void gpioButtonFxn0(uint_least8_t index){
    ButtonFlagIncrease = 1;
}

void gpioButtonFxn1(uint_least8_t index){
    ButtonFlagDecrease = 1;
}
```

## Strengths of the Project
One of the project's particular strengths was the efficient use of hardware interrupts to handle real-time changes, ensuring prompt responses to temperature fluctuations and user inputs. Additionally, code modularity and the use of descriptive naming conventions made the program highly readable and easy to follow, aiding both current understanding and future development efforts.

## Areas for Improvement
While the project was successful, there's room for improvement in error handling, particularly around failed sensor readings or hardware malfunctions. Incorporating a more robust system that can detect, log, and perhaps recover from errors could significantly enhance reliability. Additionally, expanding the system to handle more diverse scenarios, such as varying environmental conditions, could enhance its adaptability and usability.

## New Tools and Resources
Throughout this project, several valuable tools and resources were added to the support network, including:
Advanced usage of the Code Composer Studio.
In-depth knowledge of I2C protocol for sensor interactions.
Enhanced understanding of hardware interrupts and their handling in embedded systems.

## Transferable Skills
The skills honed during this project, including real-time data processing, hardware-software interfacing, and modular programming, are highly transferable. They're applicable to a wide array of projects requiring hardware interaction or real-time control, such as IoT devices, robotics, and more. These competencies are also valuable for coursework involving hardware, systems programming, or performance-critical applications.

## Maintainability, Readability, and Adaptability
This project was designed with best practices in mind to ensure maintainability, readability, and adaptability. Code was extensively commented and organized into logical modules to enhance readability. Functions were designed to have single responsibilities, promoting maintainability and future enhancements. For adaptability, hardware dependencies were isolated, and data structures were designed to allow easy adjustments, like adding new sensors or control elements.
