/*
 * animations.c
 *
 *  Created on: 24 Apr 2021
 *      Author: EA
 */
#include <Particles.h>
#include "main.h"
#include "defines.h"
#include "arm_math.h"
#include "animations.h"
//#include "assert.h"float ApproxAtan2(float32_t y, float32_t x);

#define PRODUCTION 0
//#define DBG 1
#define TESTING 0

extern uint16_t volume;
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
void wheel();
void diffusion();
void diffusion_1d();

float32_t fft_mag_max = 0;

uint_fast16_t XY(uint_fast16_t x, uint_fast16_t y);
void send_frame();
void bitmap2buffer();
void log_bin_partition(float u[]);

void HsvToRgb(uint_fast16_t hsv[], uint_fast8_t rgb_space[])
{
//	uint_fast8_t rgb_space[3];
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

void wheel()
{
	switch_flag = 0;

	while(HAL_DMA_GetState(&hdma_tim16_ch1_up) != HAL_DMA_STATE_READY);
	float32_t R_angle = 0.0;
	float32_t G_angle = 0.0;
	float32_t B_angle = 0.0;
	uint_fast8_t max_brightness = 10;
	uint_fast16_t counter = 0;

//	const int vol_samples = 100;
//	uint8_t volume_array [vol_samples];
//	int index = 0;
//	float volume_avg = 0;

	while (1)
	{
		if(switch_flag == 1)
			return;

//		volume_avg
//		HAL_Delay(10);
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

bool is_edge(uint_fast16_t x, uint_fast16_t y)
{
	if(x != 0 && x != width - 1 && y != 0 && y != height - 1)
		return false;
	else
		return true;
}

void diffusion()
{
	//Solves the two dimensional diffusion/heat equation: d( u(x,y,t) )/dt = c * laplacian( u(x,y,t) )
	//Where u(x,y,t) is the function of concentration/temperature, c is the coefficient of diffusivity
	//Numerically solving (under the explicit forward Euler method) yields a solution of the form:
	//u(x,y,t+dt) = u(x,y,t) + s( u(x+dx,y,t) + u(x-dx,y,t) + u(x,y+dy,t) + u(x,y-dy,t) - 4 * u(x,y,t) )
	//Where s = c*dt/dx^2, with dt, dx are discrete constants used under the approximation
	//Basically taking some kind of average for each point and its neighbors
	//This function solves for the Dirichlete boundary conditions (BC), where the boundaries' value is determined
	//(as opposed to Neumann BC, where the spatial derivative is determined)
	//This calculation is done for each color of the R, G, and B separately
	//Be careful not to make s too large, as it may cause instability in the solution
	switch_flag = 0;

	float floatrgb[2][nled][3] = {{{ 0.0f }}};

	uint8_t max_brightness = 50;
	uint8_t min_brightness = 200;
	constexpr float s = 0.2f;//diffusion, time and distance constants combination for math
	float x_max_boundary = .4f;//non-zero boundary conditions to prevent dark frame borders
	float y_max_boundary = .4f;
	float x_min_boundary = .4f;
	float y_min_boundary = .4f;
	uint8_t current = 1;
	uint8_t next = 0;

	uint_fast16_t hsv[3];
	hsv[S] = 255;
	hsv[V] = 255;

	uint_fast8_t temprgb[3];

	while(1)
	{
		if(switch_flag == 1)
			return;

		float bass_avg = 0;
		for(int i = 1; i < ADC_BUF_SIZE / 32+1; i++)
				bass_avg += fft_mag_dB[i] / (ADC_BUF_SIZE / 32+1);

		if(hysteresis == 1)
			if(bass_avg > 2500)
				bass_detection = 0;
			else
			{
				bass_detection = 0;
				hysteresis = 0;
			}

		if(hysteresis == 0)
			if(bass_avg > 3000)
			{
				bass_detection = 1;
				hysteresis = 1;
			}

#ifdef DBG
		//randomly decide if new point appears
		if((RNG->DR % 1001) > 950)
#endif
#ifdef TESTING
		//was there a bass threshold crossing
		if(bass_detection == 1)
#endif
		{
			volume_event = 0;
			//if so, get random location
			uint_fast16_t x = (RNG->DR >> 4) % width; // bit shift to prevent x = y if RNG hasn't updated between lines
			uint_fast16_t y = RNG->DR % height;
			{
//				uint8_t color = RNG->DR % 3;// random color
				hsv[H] = RNG->DR % 255;
				HsvToRgb(hsv, temprgb);

//				floatrgb[current][XY(x,y)][color] += (RNG->DR % max_brightness)+min_brightness;// random concentration
				for(int color = 0; color < 3; color++)
					floatrgb[current][XY(x,y)][color] = (float) temprgb[color];
			}
		}
		for (uint_fast16_t x = 0; x < width; x++)
		{
			const auto is_x_middle = (1 <= x && x <= width - 1);
			const auto is_x_min = x == 0;
			const auto is_x_max = x == (width - 1);
			for (uint_fast16_t y = 0; y < height; y++)
			{
				const int xy = XY(x, y);
				const auto is_y_middle = (1 <= y && y <= height - 1);
				const auto is_y_min = (y == 0);
				const auto is_y_max = (y == height - 1);



				for (uint8_t color = 0; color < 3; color++)
				{
					float present = floatrgb[current][xy][color];
					float& future = floatrgb[next][xy][color];

					const auto t1 = !is_x_min ? floatrgb[current][XY(x - 1, y)][color] : x_min_boundary;
					const auto t2 = !is_x_max ? floatrgb[current][XY(x + 1, y)][color] : x_max_boundary;
					const auto t3 = !is_y_min ? floatrgb[current][XY(x, y - 1)][color] : y_min_boundary;
					const auto t4 = !is_y_max ? floatrgb[current][XY(x, y + 1)][color] : y_max_boundary;

					future = present * (1 - 4 * s) + s * (t1 + t2 + t3 + t4);
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


void diffusion_1d()
{
	//Solves the two dimensional diffusion/heat equation: d( u(x,y,t) )/dt = c * laplacian( u(x,y,t) )
	//Where u(x,y,t) is the function of concentration/temperature, c is the coefficient of diffusivity
	//Numerically solving (under the explicit forward Euler method) yields a solution of the form:
	//u(x,y,t+dt) = u(x,y,t) + s( u(x+dx,y,t) + u(x-dx,y,t) + u(x,y+dy,t) + u(x,y-dy,t) - 4 * u(x,y,t) )
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
		if(switch_flag == 1)
			return;

		float bass_avg = 0;
		for(int i = 1; i < ADC_BUF_SIZE/2; i++)
				bass_avg += fft_mag_dB[i] / (ADC_BUF_SIZE/2);


			if(bass_avg > threshhold)
				bass_detection = 1;
			else
				bass_detection = 0;
		//randomly decide if new point appears
//		if((RNG->DR % 1001) > 950)
		if(bass_detection == 1)
		{
			volume_event = 0;
			//if so, get random location
			uint_fast16_t x = (RNG->DR >> 4) % width; // bit shift to prevent x = y if RNG hasn't updated between lines
			uint_fast16_t y = RNG->DR % height;
			{
//				uint8_t color = RNG->DR % 3;// random color
				uint8_t tmp[3] = {0};
				hsv[H] = RNG->DR % 255;
				HsvToRgb(hsv, temprgb);

//				floatrgb[current][XY(x,y)][color] += (RNG->DR % max_brightness)+min_brightness;// random concentration
				for(int color = 0; color < 3; color++)
					floatrgb[current][XY(x,y)][color] = (float) temprgb[color];
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

void arctic_monkeys()
{
	float32_t floatrgb[nled][3] = {{ 0.0f }};
	float32_t amplitude = height;
	float counter = 0;
	while(1)
	{
		for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++)
			{
				int xy = XY(x, y);
				float xy_d;
//				arm_abs_f32(pSrc, pDst, blockSize)
				rgb[xy][R] = arm_sin_f32( 3.14 * ((float32_t) x - width/2) / width*2 )*height/counter + y > 0 ? 1 : 0;
//				rgb[xy][B] = arm_sin_f32(counter + 3.14 * ((float32_t) x) / width*2)*height + y < 0 ? 1 : 0;
			}
		counter += 0.1;

		send_frame();
	}
}

//fast two arguman arctan approximation
float ApproxAtan2(float32_t y, float32_t x)
{
    const float32_t n1 = 0.97239411f;
    const float32_t n2 = -0.19194795f;
    float32_t result = 0.0f;
    if (x != 0.0f)
    {
        const union { float32_t flVal; uint_fast32_t nVal; } tYSign = { y };
        const union { float32_t flVal; uint_fast32_t nVal; } tXSign = { x };
        if (fabsf(x) >= fabsf(y))
        {
            union { float32_t flVal; uint_fast32_t nVal; } tOffset = { PI };
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
            union { float32_t flVal; uint_fast32_t nVal; } tOffset = { PI/2 };
            // Add or subtract PI/2 based on y's sign.
            tOffset.nVal |= tYSign.nVal & 0x80000000u;
            result = tOffset.flVal;
            const float32_t z = x / y;
            result -= (n1 + n2 * z * z) * z;
        }
    }
    else if (y > 0.0f)
    {
        result = PI/2;
    }
    else if (y < 0.0f)
    {
        result = -PI/2;
    }
    return result;
}





void spectrogram()
{

	switch_flag = 0;

//	float Fs = 80000000 / ( (TIM2->PSC+1) * (TIM2->ARR+1) );

	uint_fast8_t temprgb[3];

	int linstep = (ADC_BUF_SIZE / 2) / width;
	linstep = 2;
	float bin_fill[width] = {0};
	while(1)
	{
		if( __builtin_expect(switch_flag, 0) )
			return;

		for(int i = 1; i < width+1; i++)
		  for(int j = i*linstep; j < (i+1)*linstep; j++)
			  bin_fill[i-1] +=  fft_mag_dB[j];
//		  bin_fill[width-1] = fft_mag_dB[width - 1];

		for(int x = 0; x < width; x++)
		  for(int y = 0; y < height; y++)
		  {
			  float val = log10(bin_fill[x]);
//				  float eq;
//				  arm_sqrt_f32(x+1, &eq);
			  uint_fast16_t hsv[3];
			  hsv[S] = 255;
			  hsv[V] = 15;
			  if((uint16_t) (19*(val-2.5)) > y)
			  {
//					  rgb[XY(x,y)][R] = 3;
//					  rgb[XY(x,y)][G] = (y/2 > 6 ? 0 : y/2);
//					  rgb[XY(x,y)][B] = (5-y >= 0 ? 5-y : 0);
				  hsv[H] = (y*(x+1)+x-y)%256;
				  HsvToRgb(hsv, temprgb);//&rgb[XY(x,y)]);
				  rgb[XY(x,y)][R] = temprgb[R];
				  rgb[XY(x,y)][G] = temprgb[G];
				  rgb[XY(x,y)][B] = temprgb[B];
			  }
			  else
			  {
				  rgb[XY(x,y)][R] = 0;
				  rgb[XY(x,y)][G] = 0;
				  rgb[XY(x,y)][B] = 0;
			  }
		  }
		  for(int i = 0; i < width; i++)
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

void waterfall()
{
	switch_flag = 0;

//lin_step = 2
//	float bin_avg[width] = {2.7277317, 2.56138897, 2.52718043, 2.47239828,
//			2.47640753, 2.43758774, 2.37323284, 2.41680765,
//			2.31911659, 2.28132343, 2.25431919, 2.25722528,
//			2.26941729, 2.43082142, 2.20516634, 2.29801536,
//			2.39601254, 2.177598, 2.30320096, 2.25884318,
//			2.12879729, 2.10871935, 2.08889675, 2.1518724,
//			2.4533484, 2.19418073, 2.20496607, 2.42618227,
//			2.14230657, 2.23000765, 2.30119109, 2.09324288};

	//lin_step = 1
//	float bin_avg[width] = {2.8290801, 2.7630868, 2.57888055, 2.60410976,
//				2.49171829, 2.49774027, 2.50008392, 2.46965885,
//				2.46984839, 2.48907685, 2.42154837, 2.45479155,
//				2.44669294, 2.43645382, 2.45146346, 2.50794673,
//				2.47349477, 2.40368104, 2.39772105, 2.31023145,
//				2.25313926, 2.27098966, 2.26089549, 2.20433521,
//				2.2053988, 2.21821404, 2.23654032, 2.33504653,
//				2.21932364, 2.15646839, 2.14748192, 2.16675186};

	//oversampling x32 linstep = 2
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
	while(1)
	{
	if(adc_dma_cmplt == 1)
		{
			adc_dma_cmplt = 0;

			arm_max_f32(fft_mag_dB, ADC_BUF_SIZE/2, &fft_mag_max, &no_use);
				noise_compensation = fft_mag_max > 2000 ? 0 : 1;

			float bin_fill[width] = { 0 };

			for(int i = 1; i < width+1; i++)//start at i=1 to avoid DC bin
			{
				for (int j = i * linstep; j < (i + 1) * linstep; j++)
					bin_fill[i - 1] += (fft_mag_dB[j]);
				bin_fill[i - 1] /= linstep;
			}

//			float temp_line[width] = &bin_fill[height-1];

			for(int color = 0; color < 3; color++)
				for(int x = 0; x < width; x++)
					for(int y = height - 1; y > 0; y--)
						rgb[XY(x, y)][color] = rgb[XY(x, y - 1)][color];

//			arm_max_f32(bin_fill, width, pResult, pIndex);
//			arm_min_f32(bin_fill, blockSize, pResult, pIndex)
			for(int x = 0; x < width; x++)
			{
			  float val = log10(bin_fill[x]);
			  if(noise_compensation == 1)
				  bin_avg[x] = 0.95*bin_avg[x]+0.05*val;
			  val -= bin_avg[x] + 0.2;
			  {
				  hsv[V] = val > 0 ? (val)*10 : 1;
				  hsv[H] = val > 0 ?(uint16_t)((val+1)*(val+1)*(val+1)*15) : 0;
				  HsvToRgb(hsv, temprgb);//&rgb[XY(x,0)]);
				  rgb[XY(x,0)][R] = temprgb[R];
				  rgb[XY(x,0)][G] = temprgb[G];
				  rgb[XY(x,0)][B] = temprgb[B];
			  }
			}
//			printf(" %f \n\r",log10(bin_fill[1]));
//			printf("aa \n\r");

		}

//	for(int i = 0; i < width; i++)
//		  bin_fill[i] = 0;

	bitmap2buffer();
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

void The_Matrix()
{

}


