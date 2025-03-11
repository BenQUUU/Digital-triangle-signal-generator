# Digital Triangle Waveform Generator

## Project Description
This project implements a **digital triangular waveform generator** using the STM32 microcontroller. The waveform is generated using **TIM2 (Ch1) / TIM5 (Ch1)** with configurable parameters:
- **Frequency:** Selectable between 5 Hz, 10 Hz, and 20 Hz.
- **Amplitude:** Adjustable between 10% and 90%.
- **Phase Shift:** Adjustable between 5% and 95% using a potentiometer.
- **PWM Frequency:** Fixed at 25 kHz.

The generated signal is processed using a filter to obtain an **analog output**.

## Features
- **User Interface:** A touchscreen interface is used to:
  - Enable/disable waveform generation.
  - Adjust frequency, amplitude, and phase shift.
  - Display the current waveform settings.
- **PWM Generation:** The waveform is generated using **Pulse Width Modulation (PWM)** and converted to an analog signal via filtering.

## System Overview
The system consists of the following components:
- **STM32 Microcontroller** (TIM2 Ch1 / TIM5 Ch1)
- **PWM Signal Generator**
- **Low-Pass Filter** for analog signal output
- **Touchscreen Panel** for user input
- **Potentiometer** for phase shift adjustment

## Requirements
To run this project, you need:
- STM32 microcontroller (compatible with TIM2 and TIM5)
- Touchscreen display module
- Low-pass filter circuit
- Potentiometer for phase shift control
- Embedded C development environment (STM32CubeIDE, Keil, or similar)

## Installation & Usage
1. **Flash the firmware** onto the STM32 microcontroller.
2. **Connect the hardware** as per the schematic.
3. **Use the touchscreen** to adjust waveform settings.
4. The analog triangular waveform will be outputted based on the selected parameters.

## Usage
<p align="center">
  <img src="https://imgur.com/a/mSVBKxJ" alt="Working system">
</p>

<p align="center">
  <img src="https://imgur.com/a/WUiSe4I" alt="Oscilloscope output 1">
</p>

<p align="center">
  <img src="https://imgur.com/a/6OgwV2T" alt="Oscilloscope output 2">
</p>





