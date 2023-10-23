# README for Thermostat Control System

## Introduction
This project is a thermostat control system designed for embedded platforms. The primary goal is to maintain a user-defined temperature setpoint by actively reading current temperature and toggling a heating element (simulated by an LED) on or off.

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

## Main Goal
The primary objective of this project is to demonstrate real-time hardware interaction and control using embedded systems. This is showcased through precise temperature control via sensor feedback and user input.


