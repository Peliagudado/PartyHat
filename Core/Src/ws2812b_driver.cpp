/*
 * ws2812b_driver.cpp
 *
 *  Created on: May 8, 2021
 *      Author: EA
 */
#include "main.h"
#include "defines.h"


extern DMA_HandleTypeDef hdma_tim16_ch1_up;
extern TIM_HandleTypeDef htim16;
uint8_t frame[8 * 3 * nled + extra_buffer] = { 0 }; //PWM DMA buffer, in CCR values
extern uint_fast8_t rgb[nled][3]; //RGB frame buffer

/*
 * @brief: converts an 24 bit RGB array into a timer buffer
 * @extended summary:The timer needs to receive a buffer with the data that will yield the correct OCC
 * periods according to the WS2812B LEDs data sheet
 * @effects: changes the frame buffer, which is later sent to the timer in DMA mode
 * @return: none
 */
void bitmap2buffer()
{


	for (int led_adress = 0; led_adress < nled; led_adress++)
		for (int color = 0; color < 3; color++)
		{
			uint_fast8_t temp = rgb[led_adress][color];
			for (int color_bit = 0; color_bit < 8; color_bit++)
			{
//				bool is_high = (temp << color_bit) & 0x80;
				frame[color_bit + 8 * color + 24 * led_adress] = (uint_fast8_t) (  (temp << color_bit) & 0x80 ? 70 : 20);
			}
		}
}

void send_frame()
{
	while (HAL_DMA_GetState(&hdma_tim16_ch1_up) != HAL_DMA_STATE_READY);
	bitmap2buffer();

	TIM16->CNT = 0;

	if (HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)frame, nled * 8 * 3 + extra_buffer) != HAL_OK)
		Error_Handler();
}

void reset_rgb()
{
	//Sets the RGB frame buffer to dark
	for (int led_adress = 0; led_adress < nled; led_adress++)
		for (int color = 0; color < 3; color++)
			rgb[led_adress][color] = 0;
}
