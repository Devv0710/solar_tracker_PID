/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Motor_Id {
	MOTOR_ELEVATION, MOTOR_AZIMUTH
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIEMPO_MUESTREO 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const uint8_t step_sequence[8][4] = { { 1, 0, 0, 0 }, { 1, 1, 0, 0 }, { 0, 1, 0,
		0 }, { 0, 1, 1, 0 }, { 0, 0, 1, 0 }, { 0, 0, 1, 1 }, { 0, 0, 0, 1 }, {
		1, 0, 0, 1 } };

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = { .name = "controlTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for motorAzTask */
osThreadId_t motorAzTaskHandle;
const osThreadAttr_t motorAzTask_attributes = { .name = "motorAzTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for motorElTask */
osThreadId_t motorElTaskHandle;
const osThreadAttr_t motorElTask_attributes = { .name = "motorElTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for motorAzQueue */
osMessageQueueId_t motorAzQueueHandle;
const osMessageQueueAttr_t motorAzQueue_attributes = { .name = "motorAzQueue" };
/* Definitions for motorElQueue */
osMessageQueueId_t motorElQueueHandle;
const osMessageQueueAttr_t motorElQueue_attributes = { .name = "motorElQueue" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void setStep(uint8_t step, enum Motor_Id motor_id);
uint32_t leer_adc(uint32_t channel);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ControlTask(void *argument);
void MotorAzTask(void *argument);
void MotorElTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of motorAzQueue */
	motorAzQueueHandle = osMessageQueueNew(5, sizeof(int32_t),
			&motorAzQueue_attributes);

	/* creation of motorElQueue */
	motorElQueueHandle = osMessageQueueNew(5, sizeof(int32_t),
			&motorElQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of controlTask */
	controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

	/* creation of motorAzTask */
	motorAzTaskHandle = osThreadNew(MotorAzTask, NULL, &motorAzTask_attributes);

	/* creation of motorElTask */
	motorElTaskHandle = osThreadNew(MotorElTask, NULL, &motorElTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
 * @brief Function implementing the controlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument) {
	/* USER CODE BEGIN ControlTask */
	int32_t pasos_az_prueba = 20000;
	int32_t pasos_el_prueba = 10000;

	// --- PID para Azimuth ---
	float Kp_az = 0.1;
	float Ki_az = 0.01;
	float Kd_az = 0.05;
	float integral_sum_az = 0.0;
	float last_error_az = 0.0;

	// --- PID para Elevación ---
	float Kp_el = 0.1;
	float Ki_el = 0.01;
	float Kd_el = 0.05;
	float integral_sum_el = 0.0;
	float last_error_el = 0.0;

	const float dt = 0.05; // Tiempo de muestreo = 50ms = 0.05s
	const int32_t NOCHE_THRESHOLD = 500; // Valor ADC (0-4095) que consideras "noche"

	static char tx_buffer[256];

	/* Infinite loop */
	for (;;) {
		uint32_t ldr_norte = leer_adc(ADC_CHANNEL_9);
		uint32_t ldr_sur = leer_adc(ADC_CHANNEL_10);
		uint32_t ldr_este = leer_adc(ADC_CHANNEL_11);
		uint32_t ldr_oeste = leer_adc(ADC_CHANNEL_12);
		uint32_t ldr_ref = leer_adc(ADC_CHANNEL_13);
		// 2. LÓGICA DE DORMIR
//		if (ldr_ref < NOCHE_THRESHOLD) {
//			// Es de noche. No hagas nada con el PID.
//			// (Opcional: enviar motores a posición "dormir")
//			osDelay(1000); // Dormir 1 segundo y volver a chequear
//			continue;      // Saltar el resto del bucle
//		}
		// 2. Calcular Error_Az y Error_El
		int32_t error_az = ldr_este - ldr_oeste;
		int32_t error_el = ldr_norte - ldr_sur;

		//PID AZIMUTH
		float p_term_az = Kp_az * (float) error_az;
		integral_sum_az += (float) error_az * dt;
		float i_term_az = Ki_az * integral_sum_az;
		float error_deriv_az = ((float) error_az - last_error_az) / dt;
		float d_term_az = Kd_az * error_deriv_az;
		last_error_az = (float) error_az;

		//PID ELEVACIÓN
		float p_term_el = Kp_el * (float) error_el;
		integral_sum_el += (float) error_el * dt;
		float i_term_el = Ki_el * integral_sum_el;
		float error_deriv_el = ((float) error_el - last_error_el) / dt;
		float d_term_el = Kd_el * error_deriv_el;
		last_error_el = (float) error_el;

		//SALIDA DEL PID AZIMUTH
		int32_t pasos_az = (int32_t) (p_term_az + i_term_az + d_term_az);
		//SALIDA DEL PID ELEVACION
		int32_t pasos_el = (int32_t) (p_term_el + i_term_el + d_term_el);

		// 5. Enviar la orden a las otras tareas (usando una "Queue" de FreeRTOS)
		if (pasos_az != 0) {
			osMessageQueuePut(motorAzQueueHandle, &pasos_az, 0U, 0U);
		}
		if (pasos_el != 0) {
			osMessageQueuePut(motorElQueueHandle, &pasos_el, 0U, 0U);
		}

		if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY) {
			// (B) Formatear el string de forma segura
			// Usamos snprintf para evitar desbordamiento del búfer
			snprintf(tx_buffer, 256,
					"E:%ld O:%ld N:%ld S:%ld | ErrAz:%ld ErrEl:%ld | P_Az:%ld P_El:%ld\r\n",
					ldr_este, ldr_oeste, ldr_norte, ldr_sur, error_az, error_el,
					pasos_az, pasos_el);

			// (C) Iniciar la transmisión DMA
			// Esta función retorna INMEDIATAMENTE (no bloquea)
			// El hardware de DMA se encarga del resto.
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*) tx_buffer,
					strlen(tx_buffer));
		}

		// 6. Dormir esta tarea por 50ms (para el bucle de control)
		osDelay(50);

//		osMessageQueuePut(motorAzQueueHandle, &pasos_az_prueba, 0U, 0U);
//		osMessageQueuePut(motorElQueueHandle, &pasos_el_prueba, 0U, 0U);
//
//		osDelay(5000);
//
//		// 3. Enviar órdenes de regreso
//		int32_t pasos_az_regreso = -20000;
//		int32_t pasos_el_regreso = -10000;
//		osMessageQueuePut(motorAzQueueHandle, &pasos_az_regreso, 0U, 0U);
//		osMessageQueuePut(motorElQueueHandle, &pasos_el_regreso, 0U, 0U);
//
//		// 4. Dormir 5 segundos
//		osDelay(5000);

	}
	/* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_MotorAzTask */
/**
 * @brief Function implementing the motorAzTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorAzTask */
void MotorAzTask(void *argument) {
	/* USER CODE BEGIN MotorAzTask */
	int32_t pasos_a_mover = 0;
	uint8_t paso_actual_az = 0;
	/* Infinite loop */
	for (;;) {
		// 1. Esperar (bloqueado) hasta recibir una orden en la cola de AZIMUTH
		if (osMessageQueueGet(motorAzQueueHandle, &pasos_a_mover, NULL,
		osWaitForever) == osOK) {

			// 2. Ejecutar la orden
			int32_t pasos_abs =
					(pasos_a_mover > 0) ? pasos_a_mover : -pasos_a_mover;

			for (int i = 0; i < pasos_abs; i++) {
				if (pasos_a_mover > 0) {
					paso_actual_az++;
					if (paso_actual_az > 7)
						paso_actual_az = 0;
				} else {
					paso_actual_az--;
					if (paso_actual_az < 0)
						paso_actual_az = 7;
				}
				setStep(paso_actual_az, MOTOR_AZIMUTH);
				osDelay(4);
			}
		}
	}
	/* USER CODE END MotorAzTask */
}

/* USER CODE BEGIN Header_MotorElTask */
/**
 * @brief Function implementing the motorElTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorElTask */
void MotorElTask(void *argument) {
	/* USER CODE BEGIN MotorElTask */
	int32_t pasos_a_mover = 0;
	uint8_t paso_actual_el = 0;
	/* Infinite loop */
	for (;;) {
		if (osMessageQueueGet(motorElQueueHandle, &pasos_a_mover, NULL,
		osWaitForever) == osOK) {
			int32_t pasos_abs =
					(pasos_a_mover > 0) ? pasos_a_mover : -pasos_a_mover;

			for (int i = 0; i < pasos_abs; i++) {
				if (pasos_a_mover > 0) {
					paso_actual_el++;
					if (paso_actual_el > 7)
						paso_actual_el = 0;
				} else {
					paso_actual_el--;
					if (paso_actual_el < 0)
						paso_actual_el = 7;
				}
				setStep(paso_actual_el, MOTOR_ELEVATION);
				osDelay(4);
			}
		}
	}
	/* USER CODE END MotorElTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void setStep(uint8_t step, enum Motor_Id motor_id) {
// Asegurarse de que el índice esté en el rango 0-7
	step = step % 8;

	switch (motor_id) {
	case MOTOR_AZIMUTH:
		HAL_GPIO_WritePin(M_AZ_1_GPIO_Port, M_AZ_1_Pin,
				step_sequence[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_AZ_2_GPIO_Port, M_AZ_2_Pin,
				step_sequence[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_AZ_3_GPIO_Port, M_AZ_3_Pin,
				step_sequence[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_AZ_4_GPIO_Port, M_AZ_4_Pin,
				step_sequence[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	case MOTOR_ELEVATION:
		HAL_GPIO_WritePin(M_EL_1_GPIO_Port, M_EL_1_Pin,
				step_sequence[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_EL_2_GPIO_Port, M_EL_2_Pin,
				step_sequence[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_EL_3_GPIO_Port, M_EL_3_Pin,
				step_sequence[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M_EL_4_GPIO_Port, M_EL_4_Pin,
				step_sequence[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

uint32_t leer_adc(uint32_t channel) {
//	ADC_ChannelConfTypeDef sConfig = { 0 };
	uint32_t adc_valor = 0;

	/*
	 */
	return adc_valor;
}

/* USER CODE END Application */

