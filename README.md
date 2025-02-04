# STM32 I2C EEPROM Communication (Bare Metal)

This project demonstrates I2C communication with an EEPROM using the STM32 microcontroller in a bare-metal implementation.

## Features
- Configures GPIOB for I2C1 communication
- Implements EEPROM read and write operations
- Uses polling method for simplicity
- Stores and retrieves data from EEPROM

## Hardware Requirements
- STM32 microcontroller (tested on STM32F4xx)
- I2C-based EEPROM
- USB-TTL converter (for debugging if required)

## Code Overview

### I2C Initialization
```c
void I2C1_Init(void) {
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB_MODER   |= (2 << (I2C1_SCL_PIN * 2)) | (2 << (I2C1_SDA_PIN * 2));
    GPIOB_OTYPER  |= (1 << I2C1_SCL_PIN) | (1 << I2C1_SDA_PIN);
    GPIOB_OSPEEDR |= (3 << (I2C1_SCL_PIN * 2)) | (3 << (I2C1_SDA_PIN * 2));
    GPIOB_PUPDR   |= (1 << (I2C1_SCL_PIN * 2)) | (1 << (I2C1_SDA_PIN * 2));
    GPIOB_AFRL    |= (4 << (I2C1_SCL_PIN * 4)) | (4 << (I2C1_SDA_PIN * 4));

    I2C1_CR1 |= (1 << 15);
    I2C1_CR1 &= ~(1 << 15);

    I2C1_CR2  = 42;
    I2C1_CCR  = 210;
    I2C1_TRISE = 43;
    I2C1_CR1 |= I2C_CR1_PE;
}
```

### EEPROM Write
```c
void EEPROM_Write(uint8_t mem_addr, char data) {
    I2C1_Start(EEPROM_ADDR, 0);
    I2C1_WriteByte(mem_addr);
    I2C1_WriteByte(data);
    I2C1_Stop();
    for (volatile int i = 0; i < 50000; i++);
}
```

### EEPROM Read
```c
uint8_t EEPROM_Read(uint8_t mem_addr) {
    char data;
    I2C1_Start(EEPROM_ADDR, 0);
    I2C1_WriteByte(mem_addr);
    I2C1_Stop();

    I2C1_Start(EEPROM_ADDR, 1);
    data = I2C1_ReadByte(0);
    I2C1_Stop();
    return data;
}
```



