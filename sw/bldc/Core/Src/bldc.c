#include "bldc.h"
#include "main.h"

// Private defines
#define PWM_FREQ            20000   // PWM frequency in Hz
#define PWM_PERIOD          (SystemCoreClock / (2 * PWM_FREQ)) // For center-aligned mode
#define ADC_SAMPLES         10      // Number of ADC samples for averaging
#define STARTUP_DUTY        20      // Startup duty cycle (%)
#define STARTUP_SPEED       500     // Startup speed (commutation delay in ms)
#define MINIMUM_SPEED       100     // Minimum speed (commutation delay in ms)
#define COMMUTATION_DELAY   30      // Delay between ADC reading and commutation (in degrees)
#define ZERO_CROSS_THRESHOLD 100    // ADC threshold for zero crossing detection


// Private variables
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

uint16_t adc_values[3];          // ADC values for each phase
uint8_t current_step = 0;        // Current commutation step
uint32_t commutation_time = 0;   // Time since last commutation
uint32_t commutation_delay = STARTUP_SPEED; // Initial commutation delay
bool motor_running = false;      // Motor running flag
bool startup_completed = false;  // Startup sequence completed flag
uint8_t duty_cycle = 0;          // PWM duty cycle (0-100%)

/**
 * Start the BLDC motor
 * initial_duty: Initial duty cycle (0-100%)
 */
void BLDC_Start(uint8_t initial_duty)
{
    if (motor_running) return;

    duty_cycle = initial_duty;
    current_step = 0;
    commutation_delay = STARTUP_SPEED;
    startup_completed = false;
    motor_running = true;

    // Enable PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // Enable timer interrupt for commutation control
    HAL_TIM_Base_Start_IT(&htim1);

    // Start forced commutation sequence
    BLDC_StartupSequence();
}

/**
 * Stop the BLDC motor
 */
void BLDC_Stop(void)
{
    // Disable PWM channels
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    // Disable ADC
    HAL_ADC_Stop_DMA(&hadc1);

    // Disable timer interrupt
    HAL_TIM_Base_Stop_IT(&htim1);

    motor_running = false;
}

/**
 * Set the PWM duty cycle
 * duty: Duty cycle (0-100%)
 */
void BLDC_SetDuty(uint8_t duty)
{
    duty_cycle = duty > 100 ? 100 : duty;
    uint32_t pulse = (PWM_PERIOD * duty_cycle) / 100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}

/**
 * Commutate the motor to the specified step
 * step: Commutation step (0-5)
 */
void BLDC_Commutate(uint8_t step)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t pulse = (PWM_PERIOD * duty_cycle) / 100;

    // Reset all channels to inactive state
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // Configure channels based on commutation step
    switch (step) {
        case STEP_1: // A-B (A high, B low, C floating)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;

        case STEP_2: // A-C (A high, C low, B floating)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case STEP_3: // B-C (B high, C low, A floating)
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case STEP_4: // B-A (B high, A low, C floating)
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;

        case STEP_5: // C-A (C high, A low, B floating)
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            break;

        case STEP_6: // C-B (C high, B low, A floating)
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            break;
    }

    // Start ADC to measure BEMF on the floating phase
    if (startup_completed) {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 3);
    }
}

/**
 * Perform startup sequence with forced commutation
 */
void BLDC_StartupSequence(void)
{
    uint32_t startup_time = HAL_GetTick();
    uint32_t step_time = startup_time;
    uint32_t acceleration_time = 2000; // 2 seconds to reach minimum speed
    uint32_t elapsed;

    // Set initial duty cycle
    BLDC_SetDuty(STARTUP_DUTY);

    // Perform initial alignment
    BLDC_Commutate(0);
    HAL_Delay(500); // Allow motor to align

    // Startup sequence with gradually increasing speed
    while (!startup_completed && motor_running) {
        if (HAL_GetTick() - step_time >= commutation_delay) {
            step_time = HAL_GetTick();

            // Move to next commutation step
            current_step = (current_step + 1) % 6;
            BLDC_Commutate(current_step);

            // Gradually decrease commutation delay (increase speed)
            elapsed = HAL_GetTick() - startup_time;
            if (elapsed < acceleration_time) {
                // Linear acceleration
                commutation_delay = STARTUP_SPEED - ((STARTUP_SPEED - MINIMUM_SPEED) * elapsed / acceleration_time);
            } else {
                commutation_delay = MINIMUM_SPEED;

                // Start BEMF sensing
                startup_completed = true;
                HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 3);
            }
        }
    }
}

/**
 * Process BEMF signals to detect zero crossing
 */
void BLDC_ProcessBEMF(void)
{
    static uint32_t last_zero_cross_time = 0;
    uint32_t current_time = HAL_GetTick();
    MotorPhase floating_phase;
    int16_t zero_cross_value;

    // Determine which phase is floating based on current step
    switch (current_step) {
        case STEP_1: floating_phase = PHASE_C; break;
        case STEP_2: floating_phase = PHASE_B; break;
        case STEP_3: floating_phase = PHASE_A; break;
        case STEP_4: floating_phase = PHASE_C; break;
        case STEP_5: floating_phase = PHASE_B; break;
        case STEP_6: floating_phase = PHASE_A; break;
        default: return;
    }

    // Check for zero crossing on the floating phase
    zero_cross_value = BLDC_GetZeroCrossing(floating_phase);

    if (abs(zero_cross_value) < ZERO_CROSS_THRESHOLD) {
        // Zero crossing detected
        if (current_time - last_zero_cross_time > 10) { // Debounce
            last_zero_cross_time = current_time;

            // Calculate commutation delay based on BEMF timing
            uint32_t period = current_time - commutation_time;

            // Apply commutation delay (typically 30 electrical degrees after zero crossing)
            uint32_t delay = (period * COMMUTATION_DELAY) / 180;

            // Schedule next commutation
            HAL_Delay(delay);

            // Perform commutation
            current_step = (current_step + 1) % 6;
            BLDC_Commutate(current_step);
            commutation_time = HAL_GetTick();
        }
    }
}

/**
 * Get zero crossing value for a specific phase
 * phase: Motor phase
 * return: Zero crossing value (signed)
 */
int16_t BLDC_GetZeroCrossing(MotorPhase phase)
{
    uint16_t virtual_neutral = (adc_values[0] + adc_values[1] + adc_values[2]) / 3;
    int16_t bemf_value = 0;

    switch (phase) {
        case PHASE_A:
            bemf_value = adc_values[0] - virtual_neutral;
            break;
        case PHASE_B:
            bemf_value = adc_values[1] - virtual_neutral;
            break;
        case PHASE_C:
            bemf_value = adc_values[2] - virtual_neutral;
            break;
    }

    // Apply direction based on current step (odd vs even steps)
    if (current_step % 2) {
        bemf_value = -bemf_value;
    }

    return bemf_value;
}

/**
 * ADC conversion complete callback
 * hadc: ADC handle
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        // Process BEMF signals
        if (startup_completed && motor_running) {
            BLDC_ProcessBEMF();
        }
    }
}

/**
 * Timer period elapsed callback
 * htim: Timer handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // Used for forced commutation during startup
        if (!startup_completed && motor_running) {
            if (HAL_GetTick() - commutation_time >= commutation_delay) {
                commutation_time = HAL_GetTick();

                // Move to next commutation step
                current_step = (current_step + 1) % 6;
                BLDC_Commutate(current_step);
            }
        }
    }
}

/**
 * LOOP
 *
 */
void BLDC_loop(void)
{
	/* gradually increase speed after startup */
	if (startup_completed && motor_running) {
		HAL_Delay(1000);

		/* Increase duty cycle if below 70% */
		if (duty_cycle < 70) {
			BLDC_SetDuty(duty_cycle + 5);
		}
	}
}
