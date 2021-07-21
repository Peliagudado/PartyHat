/*
 * animations.c
 *
 *  Created on: 24 Apr 2021
 *      Author: EA
 */
#include "Particles.h"
#include "main.h"
#include "defines.h"
#include "arm_math.h"
#include "animations.h"
#include "assert.h"

#define PRODUCTION 0
//#define DBG 1
#define TESTING 0

extern uint_fast8_t rgb[nled][3];
extern DMA_HandleTypeDef hdma_tim16_ch1_up;
extern uint8_t switch_flag;
extern uint8_t volume_event;
extern uint8_t bass_detection;
extern uint8_t adc_dma_cmplt;

extern uint8_t bass_detection;
extern uint8_t hysteresis;

extern float32_t fft_mag_dB[ADC_BUF_SIZE/2];
extern uint8_t noise_compensation;

float ApproxAtan2(float32_t y, float32_t x);
void HsvToRgb(uint_fast16_t hsv[], uint_fast8_t rgb_space[]);
void Wheel();
void Diffusion();
void Diffusion1D();

float32_t fft_mag_max = 0;

uint_fast16_t XY(uint_fast16_t x, uint_fast16_t y);
void send_frame();
void bitmap2buffer();
void log_bin_partition(float u[]);

void Wheel()
{
	/*
	 * @brief: creates three angularly dependent color gradients that change their angle with time
	 * @effects: changes the RGB buffer and calls the animation display functions
	 * @params: none
	 * @returns: none
	 */
	switch_flag = 0;

	while(HAL_DMA_GetState(&hdma_tim16_ch1_up) != HAL_DMA_STATE_READY);
	float32_t R_angle = 0.0;
	float32_t G_angle = 0.0;
	float32_t B_angle = 0.0;
	uint_fast8_t max_brightness = 10;
	uint_fast16_t counter = 0;

	while (1)
	{
		if (switch_flag == 1)
			return;

		counter+=500/50;
		R_angle = (float32_t) (counter) / (541);
		G_angle = -(float32_t) (counter) / (534);
		B_angle = (float32_t) (counter) / (545);
		for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++)
			{
				uint_fast16_t xy = XY(x, y);
				float32_t sinR_angle = arm_sin_f32(ApproxAtan2((float32_t) y - ((float32_t)height-1)/2, (float32_t) x - ((float32_t)width-1)/4) + R_angle );
				float32_t sinG_angle = arm_sin_f32(ApproxAtan2((float32_t) y - ((float32_t)height-1)/2, (float32_t) x - ((float32_t)width-1)/2) + G_angle );
				float32_t sinB_angle = arm_sin_f32(ApproxAtan2((float32_t) y - ((float32_t)height-1)/2, (float32_t) x - ((float32_t)width-1)*3/4) + B_angle);
				rgb[xy][R] = (uint_fast8_t) (max_brightness * sinR_angle * ( signbit(sinR_angle) ? -1 : 1));
				rgb[xy][G] = (uint_fast8_t) (max_brightness * sinG_angle * ( signbit(sinG_angle) ? -1 : 1));
				rgb[xy][B] = (uint_fast8_t) (max_brightness * sinB_angle * ( signbit(sinB_angle) ? -1 : 1));
			}

		send_frame();
	}
}

void Diffusion()
{
	/*
	 * @brief: numerically solves diffusion equations and displays animations
	 * @extended summary: Solves the two dimensional diffusion/heat equation: d(u(x,y,t))/dt = c * laplacian(u(x,y,t))
	 * Where u(x,y,t) is the function of concentration/temperature, c is the coefficient of diffusivity
	 * Numerically solving (under the explicit forward Euler method) yields a solution of the form:
	 * u(x,y,t+dt) = u(x,y,t) + s(u(x+dx,y,t) + u(x-dx,y,t) + u(x,y+dy,t) + u(x,y-dy,t) - 4 * u(x,y,t))
	 * Where s = c*dt/dx^2, with dt, dx are discrete constants used under the approximation
	 * Basically taking some kind of average for each point and its neighbors
	 * This function solves for a periodic boundary conditions (BC) or the Dirichlete BC,
	 * where the boundaries' value is determined
	 * (as opposed to Neumann BC, where the spatial derivative is determined)
	 * This calculation is done for each color of the R, G, and B separately
	 * Be careful not to make s too large, as it may cause instability in the solution
	 * @effects: Updates the rgb space buffer to display the diffusion and calls the frame display functions
	 * @params: none
	 * @returns: none
	 */

	switch_flag = 0;

constexpr enum
	{	DIRICHLET = 0, PERIODIC = 1} BC = PERIODIC;
	fmath x_max_boundary = .4f; //Dirichlet BC. Non-zero boundary conditions to prevent dark frame borders
	fmath y_max_boundary = .4f;
	fmath x_min_boundary = .4f;
	fmath y_min_boundary = .4f;

	float floatrgb[2][nled][3] = { { { 0.0f } } };

	constexpr fmath s = 0.2f; //diffusion, time and distance constants combination for math
	uint8_t current = 1;
	uint8_t next = 0;

	uint_fast16_t hsv[3];
	hsv[S] = 255; //max saturation
	hsv[V] = 255; //max value

	uint_fast8_t temprgb[3];

	constexpr uint_fast8_t reRandom = 6; //chosen to represent number of bits in width parameter

	while (1)
	{
		if (switch_flag == 1)
			return;
#ifdef DBG
			//randomly decide if new point appears
			const auto chance = 93;
			const auto raffle  = 100;
			if ((RNG->DR % raffle ) > chance)
#endif
#ifndef DBG
		//was there a bass threshold crossing
		if (Schmitt() == 1)
#endif
		{
			//if event condition satisfied, get random location
			uint_fast16_t x = (RNG->DR >> reRandom) % width; //bit shift to prevent x = y if RNG hasn't updated between lines
			uint_fast16_t y = RNG->DR % height;
			hsv[H] = RNG->DR % 255;
			HsvToRgb(hsv, temprgb);
			for (int color = 0; color < 3; color++)
				floatrgb[current][XY(x, y)][color] = (float) temprgb[color];
		}
		for (uint_fast16_t x = 0; x < width; x++)
		{
			const auto is_x_min = (x == 0);
			const auto is_x_max = (x == (width - 1));
			for (uint_fast16_t y = 0; y < height; y++)
			{
				const int xy = XY(x, y);
				const auto is_y_min = (y == 0);
				const auto is_y_max = (y == (height - 1));

				for (uint8_t color = 0; color < 3; color++)
				{
					if (BC == PERIODIC) //use these lines for periodic BC
					{
						x_max_boundary = floatrgb[current][XY(0, y)][color];
						x_min_boundary = floatrgb[current][XY(width - 1, y)][color];
						y_max_boundary = floatrgb[current][XY(x, 0)][color];
						y_min_boundary = floatrgb[current][XY(x, height - 1)][color];
					}
					float present = floatrgb[current][xy][color];
					float &future = floatrgb[next][xy][color];

					const auto t1 = !is_x_min ? floatrgb[current][XY(x - 1, y)][color] : x_min_boundary;
					const auto t2 = !is_x_max ? floatrgb[current][XY(x + 1, y)][color] : x_max_boundary;
					const auto t3 = !is_y_min ? floatrgb[current][XY(x, y - 1)][color] : y_min_boundary;
					const auto t4 = !is_y_max ? floatrgb[current][XY(x, y + 1)][color] : y_max_boundary;

					future = present * (1 - 4 * s) + s * (t1 + t2 + t3 + t4) -
							(present >= 0.1f && BC == PERIODIC ? 0.015f : 0); // reduce intensity globally for periodic BC
				}
			}
		}

		//floatrgb to 24bit rgb
		for (int led_adress = 0; led_adress < nled; led_adress++)
			for (uint8_t color = 0; color < 3; color++)
				rgb[led_adress][color] = (uint_fast8_t) round(floatrgb[next][led_adress][color] * 3);

		uint8_t temp = current;
		current = next;
		next = temp;

		send_frame();
	}
}

void Diffusion1D()
{
	//Solves the two dimensional diffusion/heat equation: d( u(x,y,t) )/dt = c * laplacian( u(x,y,t) )
	//Where u(x,y,t) is the function of concentration/temperature, c is the coefficient of diffusivity
	//Numerically solving (under the explicit forward Euler method) yields a solution of the form:
	//u(x,y,t+dt) = u(x,y,t) + s(u(x + dx, t) + u(x - dx, t) + u(x, t)) - 2 * u(x, t))
	//Where s = c*dt/dx^2, with dt, dx are discrete constants used under the approximation
	//Basically taking some kind of average for each point and its neighbors
	//This function solves for the Dirichlete boundary conditions (BC), where the boundaries' value is determined
	//(as opposed to Neumann BC, where the spatial derivative is determined)
	//This calculation is done for each color of the R, G, and B separately
	//Be careful not to make s too large, as it may cause instability in the solution
	switch_flag = 0;

	float floatrgb[2][nled][3] = {{{ 0.0f }}};

	constexpr uint8_t max_brightness = 50;
	constexpr uint8_t min_brightness = 200;
	constexpr float s = 0.3f;//diffusion, time and distance constants combination for math
	constexpr float y_max_boundary = 0.4f;//non-zero boundary conditions to prevent dark frame borders
	constexpr float y_min_boundary = 0.4f;
	uint8_t current = 1;
	uint8_t next = 0;

	float dimming = 0.1f;
	float threshhold = 1000;

	uint_fast16_t hsv[3];
	hsv[S] = 255;
	hsv[V] = 255;

	uint_fast8_t temprgb[3];

	while(1)
	{
		if (switch_flag == 1)
			return;

		float bass_avg = 0;
		for(int i = 1; i < ADC_BUF_SIZE/2; i++)
				bass_avg += fft_mag_dB[i] / (ADC_BUF_SIZE/2);


			if (bass_avg > threshhold)
				bass_detection = 1;
			else
				bass_detection = 0;
		//randomly decide if new point appears
//		if ((RNG->DR % 1001) > 950)
		if (bass_detection == 1)
		{
			volume_event = 0;
			//if so, get random location
			uint_fast16_t x = (RNG->DR >> 4) % width; // bit shift to prevent x = y if RNG hasn't updated between lines
			uint_fast16_t y = RNG->DR % height;
			{
//				uint8_t color = RNG->DR % 3;// random color
				hsv[H] = RNG->DR % 255;
				HsvToRgb(hsv, temprgb);

				for(int color = 0; color < 3; color++)
					floatrgb[current][XY(x, y)][color] = (float) temprgb[color];
			}
		}
		for (uint_fast16_t x = 0; x < width; x++)
		{
			for (uint_fast16_t y = 0; y < height; y++)
			{
				int xy = XY(x, y);
				for (uint8_t color = 0; color < 3; color++)
				{
					float this_pixel = floatrgb[current][xy][color];
					float *next_ptr = &(floatrgb[next][xy][color]);

					if ( y != 0 && y != height - 1 ) //case of anywhere not on the boundary
					{
						*next_ptr = this_pixel * (1 - 2 * s) + s * (
									  floatrgb[current][XY(x, y - 1)][color]
									+ floatrgb[current][XY(x, y + 1)][color]);
						*next_ptr -= dimming;
						continue;
					}
					else if ( y == 0 ) //case of y_min boundary
					{
						*next_ptr = this_pixel * (1 - 2 * s) + s * (
										y_min_boundary
										+ floatrgb[current][XY(x, y + 1)][color]);
						*next_ptr -= dimming;
						continue;
					}
					else if ( y == height - 1 ) //case of y_max boundary
					{
						*next_ptr = this_pixel * (1 - 2 * s) + s * (
									  floatrgb[current][XY(x, y - 1)][color]
									+ y_max_boundary);
						*next_ptr -= dimming;
						continue;
					}
//					*next_ptr = *next_ptr - 5;
				}
			}
		}

		//floatrgb to 24bit rgb
		for(int led_adress = 0; led_adress < nled; led_adress++)
			for(uint8_t color = 0; color < 3; color ++)
				rgb[led_adress][color] = (uint_fast8_t) round(floatrgb[next][led_adress][color]);

		uint8_t temp = current;
		current = next;
		next = temp;

		send_frame();
	}
}

void log_bin_partition(float u[])
{
    float base = 10.f;
    float a = 1;
    float b = log10(ADC_BUF_SIZE/2) / log10(base);
    float c;

    c = (b - a)/(width - 1);
    for(int i = 0; i < width ; ++i)
    {
    	float val = pow(base, a + i*c);
        *u++ = pow(base, a + i*c);
    }
}
//TODO: arctic_monkeys
void ArcticMonkeys()
{
	float32_t floatrgb[nled][3] = { { 0.0f } };
	float32_t amplitude = height;
	float counter = 0;
	while (1)
	{
		for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++)
			{
				int xy = XY(x, y);
				float xy_d;
//				arm_abs_f32(pSrc, pDst, blockSize)
				rgb[xy][R] = arm_sin_f32(2 * PI * ((float32_t) x - width / 2) / width * 2) * height / counter + y > 0 ? 1 : 0;
//				rgb[xy][B] = arm_sin_f32(counter + 3.14 * ((float32_t) x) / width*2)*height + y < 0 ? 1 : 0;
			}
		counter += 0.1f;

		send_frame();
	}
}

void spectrogram()
{

	switch_flag = 0;

	uint_fast8_t temprgb[3];

	int linstep = (ADC_BUF_SIZE / 2) / width;
	linstep = 2;
	float bin_fill[width] = { 0 };
	while (1)
	{
		if (__builtin_expect(switch_flag, 0))
			return;

		for (int i = 1; i < width + 1; i++)
			for (int j = i * linstep; j < (i + 1) * linstep; j++)
				bin_fill[i - 1] += fft_mag_dB[j];

		for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++)
			{
				float val = log10(bin_fill[x]);
				uint_fast16_t hsv[3];
				hsv[S] = 255;
				hsv[V] = 15;
				if ((uint16_t) (19 * (val - 2.5f)) > y)
				{
					hsv[H] = (y * (x + 1) + x - y) % 256;
					HsvToRgb(hsv, temprgb);
					rgb[XY(x, y)][R] = temprgb[R];
					rgb[XY(x, y)][G] = temprgb[G];
					rgb[XY(x, y)][B] = temprgb[B];
				}
				else
				{
					rgb[XY(x, y)][R] = 0;
					rgb[XY(x, y)][G] = 0;
					rgb[XY(x, y)][B] = 0;
				}
			}
		for (int i = 0; i < width; i++)
			bin_fill[i] = 0;

		send_frame();
	}
}

void rainbow_update(int_fast8_t cDir[3], uint_fast8_t cCounter[3], const uint_fast8_t max_brightness)
{
	for (int color = 0; color < 3; color++)
	{
		if (nled > 1)
			for (int i = nled - 1; i > 0; i--)
				rgb[i][color] = rgb[i - 1][color];

		rgb[0][color] = (uint_fast8_t) cCounter[color];

		if (cCounter[color] == max_brightness || cCounter[color] == 0)
			cDir[color] *= -1;
		cCounter[color] += cDir[color];
	}
}

void rainbow()
{
	//Sets the first pixel to have RGB values that vary like three triangle waves between 0 and a max brightness level
	//Shifts the colors from the each pixel to the next, in order from the first to last, dumping the last pixels' value
	const uint_fast8_t max_brightness = 40;

	uint_fast8_t cCounter[3] = { 1, max_brightness * 2 / 3, max_brightness * 2 / 3 };
	int_fast8_t cDir[3] = { -1, -1, 1 };

	while (1)
	{
		rainbow_update(cDir, cCounter, max_brightness);

		bitmap2buffer();
		send_frame();
	}
}

void rainbow_update_HSV(uint_fast16_t *hsv)
{

//	HsvToRgb(hsv, &rgb[0]);

//	uint_fast8_t hsv_pixel[3] =


}

void Waterfall()
{
	/*
	 * @brief: displays a spectrum waterfall animation, with the hue and intensity dependent on the bin magnitude
	 * @extended summary: this function also implements some consideration to noise, starting at some value,
	 * and updating the value when the maximal intensity is below a certain threshold
	 * @effects: updates the RGB space buffer and calls the animation display functions
	 * @params: none
	 * @returns: none
	 */
	switch_flag = 0;

	//initial guess for noise content in each bin
	float bin_avg[width] = {
			2.75623202, 2.64720774, 2.61132169, 2.59654307, 2.58333349,
			2.56940031, 2.55367208, 2.56256509, 2.45848536, 2.38271737,
			2.35140896, 2.3340745, 2.2739768, 2.23496985, 2.229388,
			2.17840123, 2.1700604, 2.15447235, 2.13106441, 2.10652828,
			2.0709753, 2.06019187, 2.00137877, 1.97887695, 1.96964693,
			1.94064474, 1.91444314, 1.91801345, 1.90025687, 1.90494466,
			1.9031142, 1.87657118};

	int linstep = (ADC_BUF_SIZE / 2) / width;
	linstep = 3;

	uint_fast16_t hsv[3];
	hsv[S] = 255;

	uint_fast8_t temprgb[3];

	uint32_t no_use;
	while (1)
	{
		if (adc_dma_cmplt == 1)
		{
			adc_dma_cmplt = 0;

			arm_max_f32(fft_mag_dB, ADC_BUF_SIZE / 2, &fft_mag_max, &no_use);
			noise_compensation = fft_mag_max > 2000 ? 0 : 1; //determine if the sound is at noise levels

			float bin_fill[width] = { 0.0f };

			for (int i = 1; i < width + 1; i++) //start at i=1 to avoid DC bin
			{
				for (int j = i * linstep; j < (i + 1) * linstep; j++)
					bin_fill[i - 1] += (fft_mag_dB[j]);
				bin_fill[i - 1] /= linstep;
			}

			for (int color = 0; color < 3; color++)
				for (int x = 0; x < width; x++)
					for (int y = height - 1; y > 0; y--)
						rgb[XY(x, y)][color] = rgb[XY(x, y - 1)][color];

			for (int x = 0; x < width; x++)
			{
				float val = log10(bin_fill[x]);
				if (noise_compensation == 1)
					bin_avg[x] = 0.95f * bin_avg[x] + 0.05f * val; //if the environment is quiet enough, slowly correct current bin value
				val -= bin_avg[x] + 0.2;
				{
					hsv[V] = val > 0 ? (val) * 10 : 1;
					hsv[H] = val > 0 ? (uint16_t) ((val + 1) * (val + 1) * (val + 1) * 15) : 0;
					HsvToRgb(hsv, temprgb); //&rgb[XY(x,0)]);
					rgb[XY(x, 0)][R] = temprgb[R];
					rgb[XY(x, 0)][G] = temprgb[G];
					rgb[XY(x, 0)][B] = temprgb[B];
				}
			}
		}

		send_frame();
	}
}

void rainbow_HSV()
{
	uint_fast16_t hsv[3];
	hsv[H] = 0;
	hsv[S] = 255;
	hsv[V] = 10;

	uint_fast8_t counter = 0;

	uint_fast8_t temprgb[3];

	while(1)
	{
		if (nled > 1)
			for(int color = 0; color < 3; color++)
				for (int i = nled - 1; i > 0; i--)
					rgb[i][color] = rgb[i - 1][color];

		HsvToRgb(hsv, temprgb);

		rgb[0][R] = temprgb[R];
		rgb[0][G] = temprgb[G];
		rgb[0][B] = temprgb[B];
		bitmap2buffer();
		send_frame();
		counter++;
		counter %= 240;
		hsv[H] = counter;
	}
}

//@TODO: add Matrix like animation using particles
void The_Matrix()
{

}


