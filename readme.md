# Semtech LR1121 LoRa& LoRaWAN Solution with STM32

# Makerfabs

[Makerfabs home page](https://www.makerfabs.com/)

[Makerfabs Wiki](https://wiki.makerfabs.com/)

## Intruduce

Product Link : [Semtech LR1121 LoRa& LoRaWAN Solution with STM32](https://www.makerfabs.com/semtech-lr1121-lora-lorawan-solution-with-stm32.html)

Wiki Link : [Semtech LR1121 LoRa& LoRaWAN Solution with STM32](https://wiki.makerfabs.com/STM32_LR1121.html)

## Features

- Controller: STM32L476RGT6(Cortex-M4, 1MB Flash, 128KB SRAM).
- RF Transceiver: Semtech LR1121.
- Support Protocols: LoRa, (G)FSK, Sigfox, LR-FHSS.
- Support Frequency Bands: 150–960 MHz, 1.9-2.1GHz Satellite band and 2.4GHz ISM bands.
- Interfaces: SPI, UART, I2C, GPIO, ADC

## Usage

Before getting started, the LR1121 must be programmed with firmware. Depending on the application, two types of firmware are available: **LoRa mode** and **LoRaWAN mode**.

## Example

### LoRa

The LoRa example is a basic point-to-point communication demo. It requires two boards, one as the transmitter and the other as the receiver. The transmitter continuously sends an incrementing number, which is received and displayed by the receiver.

### Work with MOS

This example uses the LoRa protocol to communicate with the [4-Channel MOSFET Driver](https://www.makerfabs.com/lora-4-channel-mosfet-driver.html) in the MaLoRa series.
Users can send predefined commands via the serial port to control the on/off state of the MOSFET channels. Once a command is received, the MOSFET module will respond by sending the real-time control status back to the STM32.

### LoRaWAN

The LoRaWAN example reads data from a DHT11 and transmits the temperature and humidity to The Things Network (TTN) via LoRaWAN. 
