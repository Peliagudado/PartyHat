/*
 * AnimationSupport.cpp
 *
 *  Created on: 5 Jun 2021
 *      Author: EA
 */
#include "main.h"
#include "defines.h"

const uint_fast16_t XY(const uint_fast16_t x, const uint_fast16_t y)
{
	(x >= 0 && x < width);
	(y >= 0 && y < height);
	return x * height + ((x & 1) ? y : (height - 1 - y));
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