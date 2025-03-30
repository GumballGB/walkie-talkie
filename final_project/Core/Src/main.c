/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//#define AUDIO_BUFFER_SIZE 44100

#define TIMER_PERIOD (SYSTEM_FREQ / SAMPLE_RATE)  // Calculate the correct timer period


uint8_t DmaRecHalfBuffCplt=0;
uint8_t DmaRecBuffCplt=0;

uint8_t button_state = 0;

//FOR THE SPEAKER

#define GUITAR_MIN_FREQ 80   // Low E string
#define GUITAR_MAX_FREQ 400  // High E harmonic

#define WAVE_MAX_VALUE 4095 // 12-bit DAC resolution 4095
#define WAVE_HALF_VALUE 2047
#define BOOST_FACTOR 4 // You can increase this value for more amplification
#define SAMPLE_COUNT 1000  // Number of samples in one sine wave cycle
#define RECORD_TIME_SEC 1      // Record up to 3 seconds

// recall Nyquist : sample rate >= 2 * max freq
#define SAMPLE_RATE 44100 // sample rate
#define SYSTEM_FREQ 80000000
#define PI 3.141592f

//for guitar tuning
#define AUDIO_BUFFER_SIZE (SAMPLE_RATE * RECORD_TIME_SEC)  // = 132,300 samples
#define FFT_SIZE 256  // Smaller size works for guitar notes


int32_t RecBuf[AUDIO_BUFFER_SIZE];

uint16_t speakerWave[SAMPLE_RATE * 5]; // Global array to store values for the speaker
uint16_t sampleIndex = 0;

#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n))) // For SWV TraceLog (from Tut.)

//for guitar tuning
float32_t fftBuffer[FFT_SIZE];
arm_rfft_fast_instance_f32 fftHandler;
uint8_t pitchDetected = 0;
float currentPitch = 0;
char currentNote[8];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// HELPER FUNCTIONS FOR GUITAR TUNING //


// Find the loudest section of the buffer (handles decaying plucks)
uint32_t FindLoudSection(int32_t* buffer, uint32_t size) {
    int32_t maxVal = 0;
    uint32_t endIndex = size;

    // Find peak volume
    for(uint32_t i = 0; i < size; i++) {
        int32_t absVal = buffer[i] > 0 ? buffer[i] : -buffer[i]; // Manual abs
        if(absVal > maxVal) maxVal = absVal;
    }

    // Find where signal drops below 20% of peak
    int32_t threshold = maxVal / 5;
    for(uint32_t i = 0; i < size; i++) {
        if((buffer[i] > 0 ? buffer[i] : -buffer[i]) < threshold) {
            endIndex = i;
            break;
        }
    }

    // Return constrained value
    return endIndex > FFT_SIZE ? FFT_SIZE : endIndex;
}

// Autocorrelation-based pitch detection
float AutoCorrelationTune(int32_t* buffer, uint32_t size) {

    // Calculate lag bounds
    uint32_t minLag = SAMPLE_RATE / GUITAR_MAX_FREQ; // ~110 samples
    uint32_t maxLag = SAMPLE_RATE / GUITAR_MIN_FREQ; // ~551 samples

    // Safety checks
    if(size < maxLag*2) {
        UART_Print("Need 1.1k samples minimum\r\n");
        return 0;
    }

    // Convert uint16_t to float32_t (-1 to +1 range)
    float32_t floatBuf[size];
    for(uint32_t i=0; i<size; i++) {
        floatBuf[i] = (buffer[i] - 2048) / 2048.0f; // 12-bit DAC center at 2048
    }

    float maxCorrelation = -1;
    uint32_t bestLag = minLag;

    // Sample every 4th lag for speed (we can interpolate later)
    for(uint32_t lag = minLag; lag < maxLag; lag += 4) {
        float correlation = 0;
        uint32_t compareLength = size - lag;

        // Only check every 4th sample for speed
        for(uint32_t i = 0; i < compareLength; i += 4) {
            correlation += floatBuf[i] * floatBuf[i+lag];
        }

        if(correlation > maxCorrelation) {
            maxCorrelation = correlation;
            bestLag = lag;
        }
    }

    // Refine around the best lag (+-3 samples)
    maxCorrelation = -1;
    uint32_t refineStart = bestLag > 3 ? bestLag - 3 : minLag;
    uint32_t refineEnd = bestLag + 3 < maxLag ? bestLag + 3 : maxLag;

    for(uint32_t lag = refineStart; lag <= refineEnd; lag++) {
        float correlation = 0;
        for(uint32_t i = 0; i < size - lag; i++) {
            correlation += buffer[i] * buffer[i+lag];
        }

        if(correlation > maxCorrelation) {
            maxCorrelation = correlation;
            bestLag = lag;
        }
    }

    return (float)SAMPLE_RATE / bestLag;
}


// Convert frequency to note name and cents
void DisplayTuning(float freq) {

	//the guitar notes!
    const char* notes[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
    const float targets[] = {82.41, 110.0, 146.83, 196.0, 246.94, 329.63};
    char msg[32];

    // Find closest guitar string
    uint8_t closest = 0;
    float minDiff = fabsf(freq - targets[0]);
    for(uint8_t i = 1; i < 6; i++) {
        float diff = fabsf(freq - targets[i]);
        if(diff < minDiff) {
            minDiff = diff;
            closest = i;
        }
    }

    // Calculate cents (1/100th of a semitone)
    float cents = 1200 * log2f(freq / targets[closest]);


    if(fabsf(cents) < 5) {
        snprintf(msg, sizeof(msg), "%s: Perfect!\r\n", notes[closest]);
    }
    else if(cents < 0) {
        snprintf(msg, sizeof(msg), "%s: %d cents LOW\r\n",
                notes[closest], (int)fabsf(cents));
    }
    else {
        snprintf(msg, sizeof(msg), "%s: %d cents HIGH\r\n",
                notes[closest], (int)cents);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}




// HELPER FUNCTIONS FOR GUITAR TUNING

// Helper function to send strings over UART
void UART_Print(const char *message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}


// This is insanely helpful to solve a lot of problem
void UART_Print_Error(const char *message, HAL_StatusTypeDef status) {
    char err_buf[64];
    snprintf(err_buf, sizeof(err_buf), "%s (Error: %d)\r\n", message, status);
    HAL_UART_Transmit(&huart1, (uint8_t*)err_buf, strlen(err_buf), HAL_MAX_DELAY);
}

//Button Interrupt, Toggle LED for debug
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// if button is 0 -> NOT on recording mode, ready to record
	// if button is 1 -> Recording


	if (GPIO_Pin == B_BUTTON_Pin) {


        // Check if the button is pressed
        if (HAL_GPIO_ReadPin(B_BUTTON_GPIO_Port, B_BUTTON_Pin) == GPIO_PIN_SET) {

        	//ready to record. It also means we need to wipe out any recBuf
        	if (button_state == 0) {

				UART_Print("[recording...]\r\n");

                //just in case stop DMA
                //stop the timer now
                HAL_TIM_Base_Stop(&htim2);
                HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1); // Ensure DAC DMA is stopped first
                HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);  // Ensure DFSDM DMA is stopped


				//clear Buffers
                memset(RecBuf, 0, sizeof(RecBuf));  // Uncomment if needed
                memset(speakerWave, 2048, sizeof(speakerWave));  // Reset to silence (mid-scale)

//                // Reinitialize DFSDM completely
//                MX_DFSDM1_Init();
//                MX_DMA_Init(); // Reinitialize DMA as well

                //lit the LED
                HAL_GPIO_WritePin(G_LED2_GPIO_Port, G_LED2_Pin, GPIO_PIN_SET);

                //start the microphone DMA

                // Start DMA with fresh configuration
                if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuf, AUDIO_BUFFER_SIZE) != HAL_OK) {
                    UART_Print("DFSDM start failed!\r\n");
                }


                HAL_TIM_Base_Start(&htim2); // Start DAC trigger timer




        		button_state = 1;

        	}
        	else if (button_state == 1) {

        	   // Print status message
                UART_Print("[processing audio...]\r\n");

                //unlit the LED
                HAL_GPIO_WritePin(G_LED2_GPIO_Port, G_LED2_Pin, GPIO_PIN_RESET);

                // Stop the current recording before starting a new one
                HAL_DFSDM_FilterRegularStop(&hdfsdm1_filter0);

                //stop the timer now
                HAL_TIM_Base_Stop(&htim2);




//                Mic-->PDM[PDM Data in RecBuf]
//                PDM-->DFSDM[DFSDM Filter]
//                DFSDM-->PCM[PCM in speakerWave]

                // Process recorded data
                for (uint16_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {

                	// Convert to 24-bit signed
                	int32_t sample24 = RecBuf[i] >> 8;

                	 // Extract the top 12 bits
                	 int16_t sample12 = sample24 >> 12;

                	 // Apply the boost by multiplying by the boost factor
                	 int32_t boostedSample = sample12 * BOOST_FACTOR;

                	 // Ensure the value doesn't exceed DAC range (0 to 4095)
                	 if (boostedSample > 2047) {
                	     boostedSample = 2047;  // Max positive value for 12-bit DAC
                	 } else if (boostedSample < -2048) {
                	     boostedSample = -2048; // Max negative value for 12-bit DAC
                	 }

                	 // Convert the boosted value back to unsigned format for the DAC (0-4095)
                	 uint16_t dacValue = (uint16_t)(boostedSample + 2048);

                	 // Ensure the final DAC value is within the DAC's valid range
                	 if (dacValue > 4095) {
                	    dacValue = 4095; // Maximum value for 12-bit DAC
                	 } else if (dacValue < 0) {
                	    dacValue = 0; // Minimum value for 12-bit DAC
                	 }

                	 // Store the DAC value in the speakerWave buffer
                	 speakerWave[i] = dacValue;

                }


                //PROCESS AUDIO
                // Stop recording and tune

                // Problems are being caused here

                //uint32_t usableSamples = FindLoudSection(RecBuf, AUDIO_BUFFER_SIZE);

                float freq = AutoCorrelationTune(speakerWave, AUDIO_BUFFER_SIZE);

                if(freq > 70 && freq < 400) { // Valid guitar range
                    DisplayTuning(freq);

                } else {
                    UART_Print("No note detected - try again\r\n");
                }
                // ... existing playback code ...
                //DONE PROCESSING AUDIO


				UART_Print("[playing back through speaker...]\r\n");
                //just in case stop DMA
                //HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1); // Ensure DAC DMA is stopped first

                HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint16_t*)speakerWave, AUDIO_BUFFER_SIZE, DAC_ALIGN_12B_R);
                HAL_TIM_Base_Start(&htim2); // Start DAC trigger timer

                button_state = 0;

        	}

        }


	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if(htim == &htim3 && button_state == 1) {
        uint32_t usableSamples = FindLoudSection(RecBuf, AUDIO_BUFFER_SIZE);
        if(usableSamples > FFT_SIZE/2) {
            float freq = AutoCorrelationTune(RecBuf, usableSamples);
            if(freq > 70 && freq < 400) {
                DisplayTuning(freq);
            }
        }
    }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Set up FFT
	arm_rfft_fast_init_f32(&fftHandler, FFT_SIZE);

	// Set up a timer for periodic pitch detection (every 100ms)
	HAL_TIM_Base_Start_IT(&htim3);  // Using TIM3 for periodic detection
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    // Initialize UART and send welcome message
  UART_Print("\r\nAudio Recorder/Player Ready\r\n");
  UART_Print("Press button to start recording\r\n");
  UART_Print("Press button again to play back\r\n\r\n");


  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Start DAC channel 1
  HAL_TIM_Base_Start_IT(&htim2);


  memset(RecBuf, 0, sizeof(RecBuf));  // Uncomment if needed
  memset(speakerWave, 2048, sizeof(speakerWave));  // Reset to silence (mid-scale)


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	// Something very important to know is that when the microphone captures stuff, it is
	    // the DFSDM module, captures the PDM data. which is 32bits. However, it is converted
	  	// to 16-bit SIGNED PCM format, where the microphone outputs data in 2's complement.
		    // Debug output - show first sample value
    if(button_state == 1) { // Only when recording
        char dbg_msg[64];
        snprintf(dbg_msg, sizeof(dbg_msg), "First sample: %ld\r\n", RecBuf[0]);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg_msg, strlen(dbg_msg), HAL_MAX_DELAY);
    }
    HAL_Delay(500);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */
	HAL_StatusTypeDef status;
  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;

  //CHANGED to know why Filter is not starting again.
  status = HAL_DFSDM_FilterInit(&hdfsdm1_filter0);
    if(status != HAL_OK) {
        UART_Print_Error("Filter Init Failed", status);
        Error_Handler();
    }


  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;


    status = HAL_DFSDM_ChannelInit(&hdfsdm1_channel2);
    if(status != HAL_OK) {
        UART_Print_Error("Channel Init Failed", status);
        Error_Handler();
    }

    /* Associate channel with filter */
    status = HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON);
    if(status != HAL_OK) {
        UART_Print_Error("Filter Config Failed", status);
        Error_Handler();
    }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (TIMER_PERIOD - 1) * 4;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(G_LED2_GPIO_Port, G_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B_BUTTON_Pin */
  GPIO_InitStruct.Pin = B_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : G_LED2_Pin */
  GPIO_InitStruct.Pin = G_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(G_LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{

	DmaRecHalfBuffCplt=1;

}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{

	DmaRecBuffCplt=1;


}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
