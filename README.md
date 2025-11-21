# DSP with STM32 - Real-time FFT & Beat Detection

This repository contains a Digital Signal Processing (DSP) project implemented on the **STM32F4** microcontroller series (specifically tested on STM32F401RE).

The project performs real-time signal analysis on an incoming analog signal (e.g., from a Pulse Sensor or Audio input) using two simultaneous methods:
1.  **Frequency Domain:** Fast Fourier Transform (FFT) using the CMSIS-DSP library to detect the dominant frequency.
2.  **Time Domain:** Threshold-based algorithm for real-time beat detection (BPM calculation).

## 📋 Features

* **ADC with DMA:** High-efficiency analog sampling using DMA in Circular Mode.
* **CMSIS-DSP RFFT:** Implements a 1024-point Real FFT to analyze spectral content.
* **Dual-Processing Loop:**
    * **Slow Loop (~1Hz):** Processes the FFT buffer when DMA transfer is complete.
    * **Fast Loop (~100Hz):** Polls ADC values for immediate threshold-based event detection (e.g., heartbeats).
* **UART Logging:** Streams raw BPM data and FFT peak frequency to a serial terminal.
* **Visual Feedback:** Toggles the on-board LED (PA5) upon detecting a valid beat.

## 📂 Project Structure

The project is organized as follows in **STM32CubeIDE**:

```text
DSP-with-STM32
├── Core
│   ├── Inc                    # Application header files (main.h, stm32f4xx_it.h)
│   ├── Src                    # Application source files
│   │   ├── main.c             # Main program loop (FFT & Threshold logic)
│   │   ├── stm32f4xx_it.c     # Interrupt Service Routines
│   │   └── ...
│   └── Startup                # Assembly startup code for STM32F401RE
├── DSP                        # ARM CMSIS-DSP Library
│   ├── Include                # DSP Library Header files (arm_math.h, etc.)
│   └── Source                 # DSP Source Code modules
│       ├── TransformFunctions # FFT, RFFT implementation
│       ├── ComplexMathFunctions
│       ├── StatisticsFunctions
│       └── ...
├── Drivers
│   ├── CMSIS                  # Cortex-M core definitions
│   └── STM32F4xx_HAL_Driver   # STM32 Peripheral Drivers (ADC, UART, DMA, etc.)
├── Digital Signal Processing.ioc # STM32CubeMX Configuration File
└── STM32F401RETX_FLASH.ld     # Linker Script
````

### 💡 Key Directories:

  * **Core/Src/main.c**: Contains the initialization code, the main `while(1)` loop, and the core DSP logic (`ProcessFFT` function and Threshold detection).
  * **DSP/**: Contains the full source code of the CMSIS-DSP library (v1.9.0 or compatible), allowing for easier debugging and optimization compared to using a pre-compiled static library.
  * **Drivers/**: Standard HAL drivers provided by STMicroelectronics.

## 🛠 Hardware & Pinout

* **Microcontroller:** STM32F401RETx (Nucleo-F401RE recommended).
* **Clock Speed:** Configured for 16MHz HSI.

| Pin  | Function | Description |
| :--- | :--- | :--- |
| **PA1** | ADC1_IN1 | **Analog Input** (Connect your sensor signal here) |
| **PA2** | USART2_TX | **Serial Output** (Connect to PC/Terminal) |
| **PA5** | GPIO_Output | **On-board LED** (Toggles on beat detection) |

## ⚙️ Configuration Details

### Sampling
* **Sample Rate:** 1000 Hz (1 kHz).
* **Timer:** TIM2 is configured to trigger the ADC conversion.
* **FFT Size:** 1024 samples.
* **Resolution:** ~0.97 Hz per bin.

### Thresholding Logic
These defines in `main.c` control the sensitivity of the time-domain beat detector:
```c
#define UPPER_THRESHOLD   3800    /* ADC value to trigger a beat start */
#define LOWER_THRESHOLD   3200    /* ADC value to reset beat state */
#define MIN_IBI_MS        300     /* Minimum Inter-Beat Interval (debounce) */
````

## 🚀 How to Run

1.  **Hardware Setup:**

      * Connect your analog sensor signal to **PA1**.
      * Ensure the sensor voltage levels are within 0V - 3.3V (STM32 Logic).
      * Connect the board to your PC via USB.

2.  **Software Setup:**

      * Open the project in **STM32CubeIDE**.
      * Ensure the **CMSIS-DSP** library is correctly linked in your project settings.
      * Build and Flash the code to the STM32.

3.  **Monitoring:**

      * Open a Serial Terminal (e.g., PuTTY, TeraTerm, Serial Plotter).
      * **Baud Rate:** 115200 bps.
      * **Data Format:**
          * You will see real-time BPM values (integer).
          * Periodically, you will see the FFT analysis: `FFT_Peak=XX.X Hz, FFT_BPM=XX.X`.

## 📚 Theory of Operation

### 1\. FFT Processing (DMA Interrupt)

The ADC collects 1024 samples into a buffer using DMA. Once the buffer is full (approx every 1 second), the `arm_rfft_fast_f32` function converts the time-domain signal into frequency bins. The magnitude is calculated, and the bin with the highest energy (peak) is identified as the dominant frequency.

### 2\. Threshold Detector (Main Loop)

While waiting for the DMA buffer to fill, the main loop polls the ADC current value. If the signal rises above `UPPER_THRESHOLD`, a "Beat" is registered. The time difference between the current beat and the previous beat (`LastTime`) is used to calculate the instantaneous BPM.

## 📦 Dependencies

  * **STM32Cube HAL Driver**
  * **ARM CMSIS-DSP Library** (Pre-compiled lib usually required)

## 📄 License

This project is open-source. Please refer to the specific license files within the `Drivers` directory for STMicroelectronics and ARM limited licenses.
