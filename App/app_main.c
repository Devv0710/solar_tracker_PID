/*
 * app_main.c
 *
 *  Created on: Oct 25, 2025
 *      Author: Devv
 */
#include "main.h"

// Secuencia half-step (8 pasos)
const uint8_t step_sequence[8][4] = {
		{ 1, 0, 0, 0 },
		{ 1, 1, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 1, 1, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 1, 1 },
		{ 0, 0, 0, 1 },
		{ 1, 0, 0, 1 }
};

enum Motor_Id {
	MOTOR_ELEVATION, MOTOR_AZIMUTH
};


int8_t g_paso_actual_az = 0; // Posición motor Azimuth
int8_t g_paso_actual_el = 0; // Posición motor Elevación

void setStep(uint8_t step, int motor_id);
void moverMotor(int steps, uint32_t delay_ms, int motor_id);

void app_main() {
	while (1) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		moverMotor(20000, 4, MOTOR_AZIMUTH);
		HAL_Delay(1000);
		// Mover -20000 pasos (backward)
		moverMotor(-20000, 4, MOTOR_AZIMUTH);
		HAL_Delay(1000);

		// 2. Probar Motor Elevación (Vertical)
		// Mover +10000 pasos (forward)
		moverMotor(10000, 4, MOTOR_ELEVATION);
		HAL_Delay(1000);
		// Mover -10000 pasos (backward)
		moverMotor(-10000, 4, MOTOR_ELEVATION);
		HAL_Delay(1000);
	}
}

void moverMotor(int steps, uint32_t delay_ms, int motor_id) {
	int i;
	int8_t *p_paso_actual; // Puntero a la variable de estado correcta

	switch (motor_id) {
	case MOTOR_AZIMUTH:
		p_paso_actual = &g_paso_actual_az;
		break;
	case MOTOR_ELEVATION:
		p_paso_actual = &g_paso_actual_el;
		break;
	default:
	}

	int pasos_abs = (steps > 0) ? steps : -steps;

// 3. Bucle de movimiento
	for (i = 0; i < pasos_abs; i++) {

		if (steps > 0) { // 'Forward'
			(*p_paso_actual)++;
			if (*p_paso_actual > 7)
				*p_paso_actual = 0;
		} else { // 'Backward'
			(*p_paso_actual)--;
			if (*p_paso_actual < 0)
				*p_paso_actual = 7;
		}

		// 5. Mover el motor llamando a la función de bajo nivel
		setStep(*p_paso_actual, motor_id);

		// 6. Esperar el tiempo 'tp'
		HAL_Delay(delay_ms);
	}
}

void setStep(uint8_t step, int motor_id) {
	// Asegurarse de que el índice esté en el rango 0-7
	step = step % 8;

	// Selecciona los 4 pines correctos basado en el motor_id
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
