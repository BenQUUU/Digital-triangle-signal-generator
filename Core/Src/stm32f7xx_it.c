/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Zmienne globalne
extern float srednia;             // Średnia wartość ADC
extern uint16_t tablica[4];       // Tablica przechowująca odczyty ADC
extern LTDC_HandleTypeDef hLtdcHandler; // Obsługa kontrolera LTDC
extern uint8_t czestotliwoscZadana; // Żądana częstotliwość PWM
extern float amplituda;           // Amplituda sygnału PWM
extern float przesuniecie;        // Przesunięcie fazowe sygnału PWM

float aktualnaWartosc = 0.001;    // Aktualna wartość PWM
float maksymalnaWartosc = 0.999;  // Maksymalna wartość PWM
float dodanaWartosc = 0.0;        // Wartość dodawana do PWM
uint16_t indeks = 0;               // index tablicy
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc3;  // Obsługa ADC3
extern TIM_HandleTypeDef htim2;  // Obsługa TIM2
extern TIM_HandleTypeDef htim5;  // Obsługa TIM5
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_NonMaskableInt_IRQn 0 */
    /* USER CODE END W1_NonMaskableInt_IRQn 0 */
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);  // Obsługa przerwania ADC3
  /* USER CODE BEGIN ADC_IRQn 1 */
  // Odczyt danych z rejestru ADC do tablicy
  if(indeks >= 4){
    indeks = 0;
  }

   tablica[indeks] = ADC3->DR;

  indeks++;
  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);  // Obsługa przerwania TIM2
  /* USER CODE BEGIN TIM2_IRQn 1 */
  // Obliczanie wartości dodanej na podstawie żądanej częstotliwości
  dodanaWartosc = 1.0 / (25000.0 / czestotliwoscZadana);

  // Aktualizacja aktualnej wartości PWM
  aktualnaWartosc += dodanaWartosc;
  if (aktualnaWartosc > maksymalnaWartosc) {
    aktualnaWartosc = 0.001;
  }

  // Ustawienie wartości PWM dla TIM2
  TIM2->CCR1 = aktualnaWartosc * TIM2->ARR * (amplituda * 0.01);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);  // Obsługa przerwania TIM5
  /* USER CODE BEGIN TIM5_IRQn 1 */
  // Przesunięcie fazowe sygnału PWM
  float aktualnePrzesuniecie = aktualnaWartosc;
  aktualnePrzesuniecie += przesuniecie;

  if (aktualnePrzesuniecie > maksymalnaWartosc) {
    aktualnePrzesuniecie -= maksymalnaWartosc;
  }

  // Ustawienie wartości PWM dla TIM5
  TIM5->CCR4 = aktualnePrzesuniecie * TIM5->ARR * (amplituda * 0.01);
  /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles LTDC global interrupt.
  */
void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&hLtdcHandler);  // Obsługa przerwania LTDC
}
/* USER CODE END 1 */
