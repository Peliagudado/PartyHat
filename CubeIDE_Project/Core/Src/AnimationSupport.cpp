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
extern float32_t fft_mag_dB[ADC_BUF_SIZE / 2];

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

	arm_mean_f32(fft_mag_dB + 1, 5, &bass_avg);

	static uint8_t noise_compensation = ((bass_avg > threshold * 1.5) || threshold < 2500) ? 0 : 1;

	if (noise_compensation == 1)
		threshold = 0.98f * threshold + 0.02f * bass_avg;

	if (threshold < 2500)
		threshold = 2500;

	//Scmitt trigger algorithm
	if (hysteresis == 1) //if the event has already been triggered
	{
		if (bass_avg > threshold * .8f) //if we haven't crossed the lower trheshold
		{
			bass_detection = 0; //make sure we don't trip twice
		}
		else
		{
			bass_detection = 0;
			hysteresis = 0; //reset trigger
		}
	}
	if (hysteresis == 0) //if the event hasn't triggered yet
		if (bass_avg > threshold) //and if there is a trigger event happening
		{
			bass_detection = 1; //trip trigger
			hysteresis = 1; // make sure we don't trigger before the lower threshold is crossed
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
		rgb_space[R] = hsv[V];
		rgb_space[G] = t;
		rgb_space[B] = p;
		break;
	case 1:
		rgb_space[R] = q;
		rgb_space[G] = hsv[V];
		rgb_space[B] = p;
		break;
	case 2:
		rgb_space[R] = p;
		rgb_space[G] = hsv[V];
		rgb_space[B] = t;
		break;
	case 3:
		rgb_space[R] = p;
		rgb_space[G] = q;
		rgb_space[B] = hsv[V];
		break;
	case 4:
		rgb_space[R] = t;
		rgb_space[G] = p;
		rgb_space[B] = hsv[V];
		break;
	default:
		rgb_space[R] = hsv[V];
		rgb_space[G] = p;
		rgb_space[B] = q;
		break;
	}

	return;
}

float ApproxAtan2(float32_t y, float32_t x)
{
	/*
	 * @brief: fast two arguman arctan approximation
	 * @effect: none
	 * @params: y and x coordinates
	 * @returns: floating point approximation of atan2
	 * @source: based on https://www.dsprelated.com/showarticle/1052.php
	 */
	const float32_t n1 = 0.97239411f;
	const float32_t n2 = -0.19194795f;
	float32_t result = 0.0f;
	if (x != 0.0f)
	{
		const union
		{
			float32_t flVal;
			uint_fast32_t nVal;
		} tYSign = { y };
		const union
		{
			float32_t flVal;
			uint_fast32_t nVal;
		} tXSign = { x };
		if (fabsf(x) >= fabsf(y))
		{
			union
			{
				float32_t flVal;
				uint_fast32_t nVal;
			} tOffset = { PI };
			// Add or subtract PI based on y's sign.
			tOffset.nVal |= tYSign.nVal & 0x80000000u;
			// No offset if x is positive, so multiply by 0 or based on x's sign.
			tOffset.nVal *= tXSign.nVal >> 31;
			result = tOffset.flVal;
			const float32_t z = y / x;
			result += (n1 + n2 * z * z) * z;
		}
		else // Use atan(y/x) = pi/2 - atan(x/y) if |y/x| > 1.
		{
			union
			{
				float32_t flVal;
				uint_fast32_t nVal;
			} tOffset = { PI / 2 };
			// Add or subtract PI/2 based on y's sign.
			tOffset.nVal |= tYSign.nVal & 0x80000000u;
			result = tOffset.flVal;
			const float32_t z = x / y;
			result -= (n1 + n2 * z * z) * z;
		}
	}
	else if (y > 0.0f)
	{
		result = PI / 2;
	}
	else if (y < 0.0f)
	{
		result = -PI / 2;
	}
	return result;
}
