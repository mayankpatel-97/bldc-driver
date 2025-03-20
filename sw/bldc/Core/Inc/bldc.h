#ifndef BLDC_H
#define BLDC_H
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
// Motor phase definitions
typedef enum {
    PHASE_A = 0,
    PHASE_B,
    PHASE_C
} MotorPhase;

// Motor commutation step definitions
typedef enum {
    STEP_1 = 0,  // A-B
    STEP_2,      // A-C
    STEP_3,      // B-C
    STEP_4,      // B-A
    STEP_5,      // C-A
    STEP_6       // C-B
} CommutationStep;


// Function prototypes
void BLDC_Init(void);
void BLDC_Start(uint8_t initial_duty);
void BLDC_Stop(void);
void BLDC_SetDuty(uint8_t duty);
void BLDC_Commutate(uint8_t step);
void BLDC_StartupSequence(void);
void BLDC_ProcessBEMF(void);
int16_t BLDC_GetZeroCrossing(MotorPhase phase);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void BLDC_loop(void);

#endif
