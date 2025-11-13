#include "main.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"

/* --- Defines --- */
#define FFT_SIZE      1024        /* Must be a power of 2 */
#define SAMPLE_RATE   1000.0f     /* ADC Sample Rate (1kHz) */

/* Thresholding logic defines */
#define UPPER_THRESHOLD   3800    /* Start/end of a beat */
#define LOWER_THRESHOLD   3200    /* Beat relaxation point */
#define MIN_IBI_MS        300     /* Minimum Inter-Beat Interval (ms) */

/* --- Buffers & Global Variables --- */

/* ADC/DMA Buffer (16-bit unsigned) */
uint16_t adc_buffer[FFT_SIZE];

/* FFT Buffers (32-bit float) */
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE]; /* RFFT output (complex) */
float32_t fft_mag[FFT_SIZE / 2]; /* Magnitude spectrum */

/* CMSIS-DSP FFT Instance */
arm_rfft_fast_instance_f32 fft_instance;

/* Thresholding state variables */
uint32_t LastTime = 0;
uint32_t ThisTime = 0;
uint8_t BPMTiming = 0;
uint8_t BeatComplete = 0;
int BPM_Threshold = 0; /* BPM calculated from thresholding */

/* --- Function Prototypes --- */
void SystemClock_Config(void);
void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(char *s);
void ADC_Init(void);
void LED_Init(void);
void ProcessFFT(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Peripheral Initialization */
    LED_Init();
    UART_Init();
    ADC_Init();

    /* Initialize CMSIS-DSP FFT */
    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
        UART_SendString("FFT init error\r\n");
    }

    UART_SendString("System Initialized. Starting loops...\r\n");
    char tx_buffer[64];

    while (1) {
        /*
         * TASK 1: FFT Processing
         * This runs when the DMA Transfer Complete flag is set.
         * Runs at: SAMPLE_RATE / FFT_SIZE = 1000 / 1024 = ~0.97 Hz
         */
        if (DMA2->LISR & DMA_LISR_TCIF0) {
            /* Clear the DMA flag */
            DMA2->LIFCR = DMA_LIFCR_CTCIF0;

            /* Process the newly filled buffer */
            ProcessFFT();
        }


        /*
         * TASK 2: Threshold-based Beat Detection
         * This runs much faster (approx 100 Hz) to detect individual beats.
         * It polls the ADC EOC (End of Conversion) flag.
         */
        while (!(ADC1->SR & ADC_SR_EOC));

        uint16_t value = ADC1->DR; /* Read the latest sample */
        ThisTime = HAL_GetTick(); /* Get current time */

        /* Thresholding State Machine */
        if (value > UPPER_THRESHOLD) {
            /* A new peak is detected */
            if (BeatComplete) {
                /* This is a valid new beat */
                uint32_t diff = ThisTime - LastTime;
                if (diff > MIN_IBI_MS) {
                    /* Calculate BPM: (60 seconds / interval in seconds) */
                    BPM_Threshold = (int) (60.0f / ((float) diff / 1000.0f));

                    /* --- ADDED --- */
                    /* Toggle the LED on PA5 for every valid beat */
                    GPIOA->ODR ^= GPIO_ODR_OD5;
                }
                /* Reset for next beat */
                BPMTiming = 0;
                BeatComplete = 0;
            }

            /* Start timing for a new beat */
            if (!BPMTiming) {
                LastTime = ThisTime;
                BPMTiming = 1;
            }
        }

        /* If signal drops, it's the end of the pulse wave */
        if ((value < LOWER_THRESHOLD) && BPMTiming) {
            BeatComplete = 1;
        }

        /* Send the threshold-based BPM via UART */
        /* Note: This sends data at ~100Hz due to HAL_Delay(10) */
        sprintf(tx_buffer, "%d\r\n", BPM_Threshold);
        UART_SendString(tx_buffer);
        /* Wait for a single ADC conversion to complete */

        HAL_Delay(10); /* Loop runs at approx 100 Hz */
    }
}

/**
 * @brief Processes the ADC buffer to find the peak frequency using FFT.
 */
void ProcessFFT(void) {
    char buf[64];

    /* 1. Convert 12-bit ADC (uint16_t) to float (-1.0 to 1.0) */
    for (int i = 0; i < FFT_SIZE; i++) {
        /* Normalizing: (value - midpoint) / range */
        fft_input[i] = ((float32_t) adc_buffer[i] - 2048.0f) / 2048.0f;
    }

    /* 2. Perform RFFT */
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    /* 3. Calculate magnitude of complex output */
    /* fft_output has (N/2) complex pairs */
    arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);

    /* 4. Find the peak magnitude and its index */
    float32_t maxValue;
    uint32_t maxIndex;

    /* Find max, skipping DC component (index 0) */
    arm_max_f32(&fft_mag[1], (FFT_SIZE / 2) - 1, &maxValue, &maxIndex);
    maxIndex += 1; /* Adjust index since we skipped index 0 */

    /* 5. Convert index to frequency (Hz) */
    /* Frequency Resolution = Fs / N */
    float32_t freq_res = (float32_t) SAMPLE_RATE / (float32_t) FFT_SIZE;
    float32_t peak_freq = maxIndex * freq_res;

    /* 6. Convert frequency (Hz) to BPM */
    /* --- ADDED --- */
    float32_t BPM_FFT = peak_freq * 60.0f;

    /* 7. Send results via UART */
    sprintf(buf, "FFT_Peak=%.1f Hz, FFT_BPM=%.1f\r\n", peak_freq, BPM_FFT);
    UART_SendString(buf);
}

/**
 * @brief Initializes PA5 (on-board LED) as an output.
 * --- ADDED ---
 */
void LED_Init(void) {
    /* 1. Enable GPIOA Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* 2. Set PA5 to General-purpose output mode */
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->MODER |= GPIO_MODER_MODER5_0;

    /* 3. (Optional) Set to Push-Pull, No-pull */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
}

/**
 * @brief Initializes ADC1 on Channel 1 (PA1) with DMA.
 */
void ADC_Init(void) {
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* 2. Configure PA1 as Analog input */
    GPIOA->MODER |= GPIO_MODER_MODER1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;

    /* 3. Configure DMA2 Stream 0, Channel 0 */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN; /* Disable stream */
    while (DMA2_Stream0->CR & DMA_SxCR_EN)
        ; /* Wait for it to be disabled */

    DMA2_Stream0->CR = 0; /* Reset config */
    DMA2_Stream0->CR |= (0 << DMA_SxCR_CHSEL_Pos); /* Channel 0 */
    DMA2_Stream0->CR |= DMA_SxCR_MINC; /* Memory increment */
    DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0; /* 16-bit peripheral size */
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0; /* 16-bit memory size */
    DMA2_Stream0->CR |= DMA_SxCR_CIRC; /* Circular mode */
    DMA2_Stream0->CR |= DMA_SxCR_PL_1; /* High priority */

    /* Set addresses */
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) adc_buffer;
    DMA2_Stream0->NDTR = FFT_SIZE;

    /* Clear flags */
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
    DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    /* Enable DMA stream */
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    /* 4. Configure ADC1 */
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    ADC1->CR1 &= ~ADC_CR1_SCAN; /* Disable scan mode */
    ADC1->CR2 |= ADC_CR2_CONT; /* Continuous conversion mode */
    ADC1->CR2 &= ~ADC_CR2_ALIGN; /* Right alignment */

    /* Set sequence (1 conversion, Channel 1) */
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 1;

    /* Enable DMA requests */
    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_DDS; /* DMA request on each conversion */

    /* Enable ADC */
    ADC1->CR2 |= ADC_CR2_ADON;
    HAL_Delay(1); /* Wait for ADC stable */

    /* Start conversion */
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @brief Initializes USART2 (PA2-TX) at 115200 baud.
 */
void UART_Init(void) {
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* 2. Configure PA2 (TX) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER |= GPIO_MODER_MODER2_1; /* Alternate function */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2; /* High speed */
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos); /* AF7 (USART2) */

    /* 3. Configure USART2
     * PCLK1 = 8MHz (from 16MHz HSI / 2)
     * BRR = 8,000,000 / (16 * 115200) = 4.34
     * Mantissa = 4, Fraction = 0.34 * 16 = 5.44 -> 5
     * BRR = 0x45
     */
    USART2->BRR = 0x0045;

    /* 4. Enable Transmitter and USART */
    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_UE;
}

/**
 * @brief Sends a single character via UART.
 */
void UART_SendChar(char c) {
    /* Wait for Transmit Data Register to be empty */
    while (!(USART2->SR & USART_SR_TXE))
        ;

    /* Send data */
    USART2->DR = c;
}

/**
 * @brief Sends a null-terminated string via UART.
 */
void UART_SendString(char *s) {
    while (*s) {
        UART_SendChar(*s++);
    }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; /* 16MHz / 2 = 8MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number */
    /* ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
