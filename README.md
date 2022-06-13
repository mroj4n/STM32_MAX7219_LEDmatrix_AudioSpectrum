# Audio Visualizer
A code that detects Audio through a mic and displays its audio spectrum in an 8x8 LED matrix.
It is using:
- Input: MAX4466 Microphone Preamplifiers
- Output: MAX7219 LED dot matrix
- Discover Board: NUCLEO-F446RE


## Code process
- Reading ADC values
- Performing FFT on the ADCinput Array (The FFT is done using CMSIS DSP library using arm_math.h )
- Shrinking the FFT result to 8 from 1024
- Converting shrunken FFT to range 0-8



This code uses techniques demonstrated in the following tutorials:
`https://controllerstech.com/led-dot-matrix-and-stm32/ `

`https://controllerstech.com/stm32-adc-single-channel/`

