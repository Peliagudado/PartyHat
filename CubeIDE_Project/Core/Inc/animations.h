/*
 * animations.h
 *
 *  Created on: 11 May 2021
 *      Author: EA
 */

#ifndef INC_ANIMATIONS_H_
#define INC_ANIMATIONS_H_

void wheel();
void diffusion();
void diffusion_1d();
void HsvToRgb(uint_fast16_t hsv[], uint_fast8_t rgb_space[]);
bool is_edge(uint_fast16_t x, uint_fast16_t y);
bool is_corner(uint_fast16_t x, uint_fast16_t y);
void arctic_monkeys();
float ApproxAtan2(float32_t y, float32_t x);
void spectrogram();
void rainbow_update(int_fast8_t cDir[3], uint_fast8_t cCounter[3], const uint_fast8_t max_brightness);
void rainbow();
void waterfall();
void rainbow_HSV();
void reset_rgb();



#endif /* INC_ANIMATIONS_H_ */
