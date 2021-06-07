/*
 * AnimationSupport.cpp
 *
 *  Created on: 5 Jun 2021
 *      Author: EA
 */
#include "main.h"
#include "defines.h"
#include "arm_math.h"

extern uint8_t bass_detection;
extern uint8_t hysteresis;
extern float32_t fft_mag_dB[ADC_BUF_SIZE/2];

float bass_accum_l[20] = { 0 };
float bass_accum_s[2] = { 0 };

const uint_fast16_t XY(const uint_fast16_t x, const uint_fast16_t y)
{
	assert_param(x >= 0 && x < width);
	assert_param(y >= 0 && y < height);
	return x * height + ((x & 1) ? y : (height - 1 - y));
}

const uint8_t Schmitt()
{
	fmath bass_avg = 0;
	fmath volume_avg = 0;
	static fmath threshold = 3000;

	arm_mean_f32(fft_mag_dB+1, 5, &bass_avg);

	static uint8_t noise_compensation = ((bass_avg > threshold * 1.5) || threshold < 2500) ? 0 : 1;

	if (noise_compensation == 1)
		threshold = 0.98 * threshold + 0.02 * bass_avg;

	if(threshold < 2500)
		threshold = 2500;

	//Scmitt trigger algorithm
	if (hysteresis == 1)//if the event has already been triggered
	{
		if (bass_avg > threshold * .8f)//if we haven't crossed the lower trheshold
		{
			bass_detection = 0;//make sure we don't trip twice
		}
		else
		{
			bass_detection = 0;
			hysteresis = 0;//reset trigger
		}
	}
	if (hysteresis == 0)//if the event hasn't triggered yet
		if (bass_avg > threshold)//and if there is a trigger event happening
		{
			bass_detection = 1;//trip trigger
			hysteresis = 1;// make sure we don't trigger before the lower threshold is crossed
		}

	return bass_detection;
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
