/*
 * Application.cpp
 *
 *  Created on: May 8, 2021
 *      Author: EA
 */


#include "main.h"
#include "defines.h"
#include "arm_math.h"
#include <stdio.h>
#include "animations.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern RNG_HandleTypeDef hrng;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_tim16_ch1_up;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef *gHuart;


uint_fast8_t rgb[nled][3] = { 0 }; //RGB frame buffer
extern int16_t mic_buffer[ADC_BUF_SIZE];

extern float f32_blackman_harris_window_512[512];
extern float f32_blackman_window_512[512];
extern float f32_hann_window_512[512];
extern arm_rfft_fast_instance_f32 fft_handler;




void RetargetInit(UART_HandleTypeDef *huart);
void send_frame();
uint_fast16_t XY(uint_fast16_t x, uint_fast16_t y);
void HsvToRgb(uint_fast16_t *hsv, uint_fast8_t *rgb_space);
void waterfall();



void cpp_app()
{
	RetargetInit(&huart2);


	//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	//  DWT->CYCCNT = 0;
	//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	  printf("Hello and welcome to my LED matrix hat FW\n\r");
	  printf("We will see the FFT bins now\n\r");

	  if(ADC_BUF_SIZE == 512)
		  arm_rfft_512_fast_init_f32(&fft_handler);
	  else if(ADC_BUF_SIZE == 1024)
		  arm_rfft_1024_fast_init_f32(&fft_handler);
	  else if(ADC_BUF_SIZE == 256)
		  arm_rfft_256_fast_init_f32(&fft_handler);
	  else if(ADC_BUF_SIZE == 2048)
		  arm_rfft_2048_fast_init_f32(&fft_handler);
	  else if(ADC_BUF_SIZE == 128)
		  arm_rfft_128_fast_init_f32(&fft_handler);
	  else
		  arm_rfft_fast_init_f32(&fft_handler, ADC_BUF_SIZE);

	  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		  Error_Handler();

	  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)mic_buffer, (uint32_t)ADC_BUF_SIZE) != HAL_OK)
		  Error_Handler();

	  if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
		  Error_Handler();

	  reset_rgb();
	  send_frame();

	  Waterfall();

	  Diffusion();

	  diffusion_1d();

	  arctic_monkeys();

	  rainbow_HSV();



		while(1);
}




void RetargetInit(UART_HandleTypeDef *huart)
{
	gHuart=huart;
	/* Disable I/O buffering for STDOUT stream, so that *chars are sent out as soon as they are printed. */
	setvbuf(stdout,NULL, _IONBF,0);
}

int _write(int fd , char *ptr, int len)
{
	HAL_StatusTypeDef hstatus;
	if(fd==STDOUT_FILENO || fd == STDERR_FILENO)
	{
		hstatus = HAL_UART_Transmit(gHuart, (uint8_t*) ptr, len, HAL_MAX_DELAY);
		if(hstatus == HAL_OK)
			return len;
//		else
//			return EIO;
	}
//	errno = EBADF;
	return -1;
}

void HsvToRgb(uint_fast16_t hsv[], uint_fast8_t rgb_space[])
{

/*
 * @brief: This function converts a single pixel in the HSV color space to the RGB color space
 * @params: RGB space pointer and HSV space pointer
 * @returns: changes the values of the rgb_space according to the input hsv
 * @comments: doesn't check inputs, may not be safe
 * @source: modified from https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
 */
    uint8_t region, remainder, p, q, t;

    if (hsv[S] == 0)
    {
    	rgb_space[R] = hsv[V];
    	rgb_space[G] = hsv[V];
    	rgb_space[B] = hsv[V];
        return;
    }

    region = hsv[H] / 43;
    remainder = (hsv[H] - (region * 43)) * 6;

    p = (hsv[V] * (255 - hsv[S])) >> 8;
    q = (hsv[V] * (255 - ((hsv[S] * remainder) >> 8))) >> 8;
    t = (hsv[V] * (255 - ((hsv[S] * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
        	rgb_space[R] = hsv[V]; rgb_space[G] = t; rgb_space[B] = p;
            break;
        case 1:
        	rgb_space[R] = q; rgb_space[G] = hsv[V]; rgb_space[B] = p;
            break;
        case 2:
        	rgb_space[R] = p; rgb_space[G] = hsv[V]; rgb_space[B] = t;
            break;
        case 3:
        	rgb_space[R] = p; rgb_space[G] = q; rgb_space[B] = hsv[V];
            break;
        case 4:
        	rgb_space[R] = t; rgb_space[G] = p; rgb_space[B] = hsv[V];
            break;
        default:
        	rgb_space[R] = hsv[V]; rgb_space[G] = p; rgb_space[B] = q;
            break;
    }

    return;
}

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif


extern "C"
{
	void App()
	{
		cpp_app();
	}
}
