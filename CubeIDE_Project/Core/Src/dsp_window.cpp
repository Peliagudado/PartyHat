/*
 * dsp_window.c
 *
 *  Created on: 23 Apr 2021
 *      Author: EA
 */

#include "main.h"
#include "defines.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>

extern int16_t mic_buffer[ADC_BUF_SIZE];

void blackman_harris_init_f32()
{
	//achieves very small side bands, however main lobe is wide
	float A = 0.35875;
	float B = 0.48829;
	float C = 0.14128;
	float D = 0.01168;

	for(int i = 0; i < ADC_BUF_SIZE; i++)
	{
		float idx = (float) PI*i/ADC_BUF_SIZE;
		printf("%f, ", A - B*cos(2*idx) + C*cos(4*idx) - D*cos(6*idx));
		if ((i + 1) % 10 == 0)
			printf("\n\r");
	}
}

void blackman_init_f32()
{
	//very small main lobe, ~-50dB side lobe attenuation
	float A = 0.42659;
	float B = 0.49656;
	float C = 0.076849;
	float D = 0.0;

	for(int i = 0; i < ADC_BUF_SIZE; i++)
	{
		float idx = (float) PI*i/ADC_BUF_SIZE;
		printf("%f, ", A - B*cos(2*idx) + C*cos(4*idx) - D*cos(6*idx));
		if ((i + 1) % 10 == 0)
			printf("\n\r");
	}
}

void hann_init_f32()
{
	//very small main lobe, ~-50dB side lobe attenuation
	float A = 0.5;

	for(int i = 0; i < ADC_BUF_SIZE; i++)
	{
		float idx = (float) PI*i/ADC_BUF_SIZE;
		printf("%f, ", sin(idx)*sin(idx));
		if ((i + 1) % 10 == 0)
			printf("\n\r");
	}
}

float f32_hann_window_512[512] =
{
		0.000000, 0.000038, 0.000151, 0.000339, 0.000602, 0.000941, 0.001355, 0.001844, 0.002408, 0.003047,
		0.003760, 0.004549, 0.005412, 0.006349, 0.007361, 0.008447, 0.009607, 0.010841, 0.012149, 0.013530,
		0.014984, 0.016512, 0.018112, 0.019785, 0.021530, 0.023347, 0.025236, 0.027196, 0.029228, 0.031330,
		0.033504, 0.035747, 0.038060, 0.040443, 0.042895, 0.045416, 0.048005, 0.050663, 0.053388, 0.056180,
		0.059039, 0.061965, 0.064957, 0.068014, 0.071136, 0.074322, 0.077573, 0.080888, 0.084265, 0.087705,
		0.091208, 0.094771, 0.098396, 0.102082, 0.105827, 0.109631, 0.113495, 0.117416, 0.121396, 0.125432,
		0.129524, 0.133673, 0.137876, 0.142135, 0.146447, 0.150812, 0.155230, 0.159700, 0.164221, 0.168792,
		0.173414, 0.178084, 0.182803, 0.187570, 0.192384, 0.197244, 0.202150, 0.207101, 0.212096, 0.217134,
		0.222215, 0.227338, 0.232501, 0.237705, 0.242949, 0.248231, 0.253551, 0.258908, 0.264302, 0.269731,
		0.275194, 0.280692, 0.286222, 0.291785, 0.297379, 0.303004, 0.308658, 0.314341, 0.320053, 0.325791,
		0.331555, 0.337345, 0.343159, 0.348997, 0.354858, 0.360740, 0.366644, 0.372567, 0.378510, 0.384471,
		0.390449, 0.396444, 0.402455, 0.408480, 0.414519, 0.420571, 0.426635, 0.432710, 0.438795, 0.444889,
		0.450991, 0.457101, 0.463218, 0.469340, 0.475466, 0.481596, 0.487729, 0.493864, 0.500000, 0.506136,
		0.512271, 0.518404, 0.524534, 0.530660, 0.536782, 0.542899, 0.549009, 0.555111, 0.561205, 0.567290,
		0.573365, 0.579429, 0.585481, 0.591520, 0.597545, 0.603556, 0.609551, 0.615529, 0.621490, 0.627433,
		0.633356, 0.639260, 0.645142, 0.651003, 0.656841, 0.662655, 0.668445, 0.674209, 0.679948, 0.685659,
		0.691342, 0.696996, 0.702621, 0.708215, 0.713778, 0.719308, 0.724806, 0.730269, 0.735698, 0.741092,
		0.746449, 0.751769, 0.757051, 0.762295, 0.767499, 0.772663, 0.777785, 0.782866, 0.787904, 0.792899,
		0.797850, 0.802756, 0.807616, 0.812430, 0.817197, 0.821916, 0.826586, 0.831208, 0.835780, 0.840301,
		0.844770, 0.849188, 0.853553, 0.857865, 0.862124, 0.866327, 0.870476, 0.874568, 0.878604, 0.882584,
		0.886505, 0.890369, 0.894173, 0.897918, 0.901604, 0.905229, 0.908792, 0.912295, 0.915735, 0.919112,
		0.922427, 0.925678, 0.928864, 0.931986, 0.935044, 0.938035, 0.940961, 0.943820, 0.946612, 0.949337,
		0.951995, 0.954584, 0.957105, 0.959557, 0.961940, 0.964253, 0.966496, 0.968670, 0.970772, 0.972804,
		0.974764, 0.976653, 0.978470, 0.980215, 0.981888, 0.983488, 0.985016, 0.986470, 0.987851, 0.989159,
		0.990393, 0.991553, 0.992639, 0.993651, 0.994588, 0.995451, 0.996240, 0.996953, 0.997592, 0.998156,
		0.998645, 0.999059, 0.999398, 0.999661, 0.999849, 0.999962, 1.000000, 0.999962, 0.999849, 0.999661,
		0.999398, 0.999059, 0.998645, 0.998156, 0.997592, 0.996953, 0.996240, 0.995451, 0.994588, 0.993651,
		0.992639, 0.991553, 0.990393, 0.989159, 0.987851, 0.986470, 0.985016, 0.983488, 0.981888, 0.980215,
		0.978470, 0.976653, 0.974764, 0.972804, 0.970772, 0.968669, 0.966496, 0.964253, 0.961940, 0.959557,
		0.957105, 0.954584, 0.951995, 0.949337, 0.946612, 0.943820, 0.940961, 0.938035, 0.935043, 0.931986,
		0.928864, 0.925678, 0.922427, 0.919112, 0.915735, 0.912295, 0.908792, 0.905229, 0.901604, 0.897918,
		0.894173, 0.890369, 0.886505, 0.882584, 0.878604, 0.874568, 0.870476, 0.866327, 0.862124, 0.857865,
		0.853553, 0.849188, 0.844770, 0.840300, 0.835779, 0.831208, 0.826586, 0.821916, 0.817197, 0.812430,
		0.807616, 0.802755, 0.797850, 0.792899, 0.787904, 0.782866, 0.777785, 0.772662, 0.767499, 0.762295,
		0.757051, 0.751769, 0.746449, 0.741092, 0.735698, 0.730269, 0.724806, 0.719308, 0.713777, 0.708215,
		0.702621, 0.696996, 0.691342, 0.685659, 0.679948, 0.674209, 0.668445, 0.662655, 0.656841, 0.651003,
		0.645142, 0.639260, 0.633356, 0.627433, 0.621490, 0.615529, 0.609550, 0.603556, 0.597545, 0.591520,
		0.585481, 0.579429, 0.573365, 0.567290, 0.561205, 0.555111, 0.549008, 0.542899, 0.536782, 0.530660,
		0.524534, 0.518404, 0.512271, 0.506136, 0.500000, 0.493864, 0.487729, 0.481596, 0.475466, 0.469340,
		0.463218, 0.457101, 0.450991, 0.444889, 0.438794, 0.432710, 0.426635, 0.420571, 0.414519, 0.408480,
		0.402455, 0.396444, 0.390449, 0.384471, 0.378510, 0.372567, 0.366643, 0.360740, 0.354857, 0.348997,
		0.343159, 0.337345, 0.331555, 0.325791, 0.320052, 0.314341, 0.308658, 0.303004, 0.297379, 0.291785,
		0.286222, 0.280692, 0.275194, 0.269731, 0.264302, 0.258908, 0.253551, 0.248231, 0.242949, 0.237705,
		0.232501, 0.227337, 0.222215, 0.217134, 0.212096, 0.207101, 0.202150, 0.197244, 0.192384, 0.187570,
		0.182803, 0.178084, 0.173414, 0.168792, 0.164220, 0.159699, 0.155230, 0.150812, 0.146446, 0.142134,
		0.137876, 0.133673, 0.129524, 0.125432, 0.121396, 0.117416, 0.113495, 0.109631, 0.105827, 0.102081,
		0.098396, 0.094771, 0.091207, 0.087705, 0.084265, 0.080888, 0.077573, 0.074322, 0.071136, 0.068014,
		0.064956, 0.061965, 0.059039, 0.056180, 0.053388, 0.050663, 0.048005, 0.045416, 0.042895, 0.040443,
		0.038060, 0.035747, 0.033504, 0.031330, 0.029228, 0.027196, 0.025236, 0.023347, 0.021530, 0.019785,
		0.018112, 0.016512, 0.014984, 0.013530, 0.012149, 0.010841, 0.009607, 0.008447, 0.007361, 0.006349,
		0.005412, 0.004549, 0.003760, 0.003046, 0.002408, 0.001844, 0.001355, 0.000941, 0.000602, 0.000339,
		0.000151, 0.000038
};

float f32_blackman_window_512[512] =
{
		0.006879, 0.006893, 0.006936, 0.007007, 0.007107, 0.007236, 0.007393, 0.007579, 0.007793, 0.008037,
		0.008310, 0.008613, 0.008944, 0.009306, 0.009697, 0.010119, 0.010570, 0.011053, 0.011566, 0.012110,
		0.012686, 0.013293, 0.013933, 0.014605, 0.015309, 0.016047, 0.016818, 0.017623, 0.018462, 0.019336,
		0.020244, 0.021189, 0.022169, 0.023185, 0.024239, 0.025329, 0.026458, 0.027624, 0.028829, 0.030074,
		0.031358, 0.032683, 0.034048, 0.035454, 0.036903, 0.038393, 0.039927, 0.041504, 0.043124, 0.044790,
		0.046500, 0.048255, 0.050057, 0.051906, 0.053801, 0.055745, 0.057736, 0.059777, 0.061866, 0.064006,
		0.066196, 0.068437, 0.070729, 0.073073, 0.075469, 0.077918, 0.080421, 0.082977, 0.085588, 0.088254,
		0.090974, 0.093751, 0.096583, 0.099472, 0.102418, 0.105421, 0.108481, 0.111600, 0.114777, 0.118013,
		0.121307, 0.124661, 0.128074, 0.131548, 0.135081, 0.138674, 0.142328, 0.146043, 0.149818, 0.153655,
		0.157552, 0.161511, 0.165531, 0.169612, 0.173755, 0.177959, 0.182224, 0.186551, 0.190939, 0.195388,
		0.199899, 0.204470, 0.209103, 0.213795, 0.218549, 0.223362, 0.228236, 0.233169, 0.238161, 0.243212,
		0.248322, 0.253491, 0.258717, 0.264000, 0.269340, 0.274737, 0.280190, 0.285697, 0.291260, 0.296876,
		0.302546, 0.308269, 0.314044, 0.319870, 0.325746, 0.331672, 0.337647, 0.343671, 0.349741, 0.355858,
		0.362020, 0.368226, 0.374476, 0.380768, 0.387102, 0.393476, 0.399889, 0.406340, 0.412828, 0.419352,
		0.425911, 0.432502, 0.439126, 0.445781, 0.452465, 0.459177, 0.465916, 0.472681, 0.479469, 0.486281,
		0.493113, 0.499966, 0.506836, 0.513724, 0.520626, 0.527542, 0.534471, 0.541410, 0.548358, 0.555314,
		0.562275, 0.569240, 0.576208, 0.583177, 0.590144, 0.597109, 0.604070, 0.611025, 0.617972, 0.624909,
		0.631835, 0.638748, 0.645647, 0.652528, 0.659391, 0.666234, 0.673055, 0.679852, 0.686624, 0.693368,
		0.700082, 0.706766, 0.713417, 0.720033, 0.726612, 0.733153, 0.739653, 0.746112, 0.752527, 0.758896,
		0.765218, 0.771490, 0.777711, 0.783879, 0.789993, 0.796050, 0.802049, 0.807988, 0.813866, 0.819680,
		0.825429, 0.831111, 0.836724, 0.842267, 0.847739, 0.853137, 0.858460, 0.863706, 0.868873, 0.873961,
		0.878968, 0.883891, 0.888730, 0.893483, 0.898149, 0.902725, 0.907212, 0.911607, 0.915908, 0.920116,
		0.924227, 0.928242, 0.932159, 0.935976, 0.939692, 0.943307, 0.946818, 0.950226, 0.953528, 0.956724,
		0.959813, 0.962794, 0.965666, 0.968428, 0.971078, 0.973617, 0.976044, 0.978356, 0.980555, 0.982639,
		0.984608, 0.986460, 0.988196, 0.989815, 0.991315, 0.992698, 0.993962, 0.995106, 0.996131, 0.997037,
		0.997822, 0.998487, 0.999031, 0.999454, 0.999757, 0.999938, 0.999999, 0.999938, 0.999757, 0.999454,
		0.999031, 0.998487, 0.997822, 0.997037, 0.996131, 0.995106, 0.993962, 0.992698, 0.991315, 0.989815,
		0.988196, 0.986460, 0.984608, 0.982639, 0.980555, 0.978356, 0.976043, 0.973617, 0.971078, 0.968427,
		0.965666, 0.962794, 0.959813, 0.956724, 0.953528, 0.950226, 0.946818, 0.943307, 0.939692, 0.935976,
		0.932159, 0.928242, 0.924227, 0.920116, 0.915908, 0.911607, 0.907212, 0.902725, 0.898149, 0.893483,
		0.888730, 0.883891, 0.878968, 0.873961, 0.868873, 0.863706, 0.858459, 0.853137, 0.847739, 0.842267,
		0.836724, 0.831111, 0.825429, 0.819680, 0.813866, 0.807988, 0.802049, 0.796050, 0.789993, 0.783879,
		0.777711, 0.771490, 0.765217, 0.758896, 0.752527, 0.746112, 0.739653, 0.733153, 0.726612, 0.720033,
		0.713417, 0.706766, 0.700082, 0.693368, 0.686624, 0.679852, 0.673055, 0.666234, 0.659391, 0.652528,
		0.645647, 0.638748, 0.631835, 0.624909, 0.617972, 0.611025, 0.604070, 0.597109, 0.590144, 0.583176,
		0.576208, 0.569240, 0.562275, 0.555314, 0.548358, 0.541410, 0.534471, 0.527542, 0.520626, 0.513723,
		0.506836, 0.499965, 0.493113, 0.486281, 0.479469, 0.472681, 0.465916, 0.459177, 0.452465, 0.445781,
		0.439126, 0.432502, 0.425911, 0.419352, 0.412828, 0.406340, 0.399889, 0.393476, 0.387102, 0.380768,
		0.374476, 0.368226, 0.362020, 0.355858, 0.349741, 0.343671, 0.337647, 0.331672, 0.325746, 0.319869,
		0.314043, 0.308269, 0.302546, 0.296876, 0.291260, 0.285697, 0.280190, 0.274737, 0.269340, 0.264000,
		0.258717, 0.253491, 0.248322, 0.243212, 0.238161, 0.233168, 0.228235, 0.223362, 0.218548, 0.213795,
		0.209103, 0.204470, 0.199899, 0.195388, 0.190939, 0.186551, 0.182224, 0.177959, 0.173755, 0.169612,
		0.165531, 0.161511, 0.157552, 0.153655, 0.149818, 0.146043, 0.142328, 0.138674, 0.135081, 0.131548,
		0.128074, 0.124661, 0.121307, 0.118013, 0.114777, 0.111600, 0.108481, 0.105421, 0.102418, 0.099472,
		0.096583, 0.093751, 0.090974, 0.088254, 0.085588, 0.082977, 0.080421, 0.077918, 0.075469, 0.073073,
		0.070729, 0.068437, 0.066196, 0.064006, 0.061866, 0.059777, 0.057736, 0.055745, 0.053801, 0.051906,
		0.050057, 0.048255, 0.046500, 0.044789, 0.043124, 0.041504, 0.039927, 0.038393, 0.036903, 0.035454,
		0.034048, 0.032683, 0.031358, 0.030074, 0.028829, 0.027624, 0.026458, 0.025329, 0.024239, 0.023185,
		0.022169, 0.021189, 0.020244, 0.019336, 0.018462, 0.017623, 0.016818, 0.016047, 0.015309, 0.014605,
		0.013933, 0.013293, 0.012686, 0.012110, 0.011566, 0.011053, 0.010570, 0.010119, 0.009697, 0.009306,
		0.008944, 0.008613, 0.008310, 0.008037, 0.007793, 0.007579, 0.007393, 0.007236, 0.007107, 0.007007,
		0.006936, 0.006893,
};

float f32_blackman_harris_window_512[512] =
{
		0.000060, 0.000062, 0.000069, 0.000079, 0.000094, 0.000114, 0.000138, 0.000166, 0.000200, 0.000238,
		0.000281, 0.000329, 0.000383, 0.000442, 0.000507, 0.000579, 0.000656, 0.000741, 0.000832, 0.000931,
		0.001037, 0.001151, 0.001274, 0.001405, 0.001546, 0.001696, 0.001857, 0.002028, 0.002210, 0.002403,
		0.002609, 0.002827, 0.003059, 0.003305, 0.003565, 0.003839, 0.004130, 0.004437, 0.004761, 0.005103,
		0.005463, 0.005842, 0.006241, 0.006661, 0.007102, 0.007565, 0.008051, 0.008561, 0.009096, 0.009656,
		0.010242, 0.010856, 0.011498, 0.012169, 0.012870, 0.013601, 0.014365, 0.015161, 0.015991, 0.016856,
		0.017757, 0.018694, 0.019669, 0.020682, 0.021736, 0.022830, 0.023966, 0.025145, 0.026368, 0.027636,
		0.028950, 0.030311, 0.031721, 0.033179, 0.034688, 0.036249, 0.037862, 0.039528, 0.041250, 0.043027,
		0.044861, 0.046752, 0.048703, 0.050714, 0.052786, 0.054920, 0.057117, 0.059378, 0.061704, 0.064097,
		0.066557, 0.069085, 0.071682, 0.074349, 0.077087, 0.079897, 0.082780, 0.085737, 0.088768, 0.091875,
		0.095057, 0.098317, 0.101655, 0.105070, 0.108566, 0.112141, 0.115796, 0.119533, 0.123351, 0.127252,
		0.131235, 0.135302, 0.139453, 0.143687, 0.148007, 0.152411, 0.156900, 0.161475, 0.166136, 0.170882,
		0.175714, 0.180633, 0.185637, 0.190728, 0.195905, 0.201168, 0.206516, 0.211950, 0.217470, 0.223075,
		0.228764, 0.234538, 0.240396, 0.246337, 0.252361, 0.258467, 0.264655, 0.270924, 0.277272, 0.283700,
		0.290207, 0.296790, 0.303451, 0.310186, 0.316996, 0.323879, 0.330833, 0.337859, 0.344953, 0.352115,
		0.359344, 0.366638, 0.373994, 0.381413, 0.388891, 0.396428, 0.404021, 0.411668, 0.419369, 0.427120,
		0.434920, 0.442766, 0.450657, 0.458590, 0.466564, 0.474575, 0.482622, 0.490703, 0.498814, 0.506953,
		0.515118, 0.523307, 0.531516, 0.539744, 0.547987, 0.556243, 0.564508, 0.572781, 0.581059, 0.589338,
		0.597615, 0.605889, 0.614155, 0.622412, 0.630655, 0.638882, 0.647090, 0.655276, 0.663436, 0.671569,
		0.679669, 0.687736, 0.695764, 0.703752, 0.711696, 0.719593, 0.727439, 0.735232, 0.742969, 0.750646,
		0.758260, 0.765808, 0.773287, 0.780693, 0.788025, 0.795278, 0.802449, 0.809536, 0.816535, 0.823444,
		0.830259, 0.836977, 0.843596, 0.850113, 0.856524, 0.862827, 0.869019, 0.875098, 0.881060, 0.886903,
		0.892624, 0.898221, 0.903691, 0.909032, 0.914241, 0.919316, 0.924254, 0.929054, 0.933712, 0.938228,
		0.942598, 0.946821, 0.950894, 0.954817, 0.958586, 0.962201, 0.965659, 0.968959, 0.972099, 0.975078,
		0.977895, 0.980548, 0.983035, 0.985356, 0.987510, 0.989495, 0.991311, 0.992957, 0.994431, 0.995734,
		0.996864, 0.997821, 0.998605, 0.999215, 0.999651, 0.999913, 1.000000, 0.999913, 0.999651, 0.999215,
		0.998605, 0.997821, 0.996864, 0.995734, 0.994431, 0.992957, 0.991311, 0.989495, 0.987510, 0.985356,
		0.983035, 0.980548, 0.977895, 0.975078, 0.972099, 0.968959, 0.965659, 0.962201, 0.958586, 0.954816,
		0.950894, 0.946821, 0.942598, 0.938228, 0.933712, 0.929054, 0.924254, 0.919316, 0.914241, 0.909032,
		0.903691, 0.898221, 0.892624, 0.886903, 0.881060, 0.875098, 0.869019, 0.862827, 0.856524, 0.850113,
		0.843596, 0.836977, 0.830259, 0.823444, 0.816535, 0.809536, 0.802449, 0.795278, 0.788025, 0.780693,
		0.773287, 0.765808, 0.758260, 0.750646, 0.742969, 0.735232, 0.727439, 0.719593, 0.711696, 0.703752,
		0.695764, 0.687735, 0.679669, 0.671568, 0.663436, 0.655276, 0.647090, 0.638882, 0.630655, 0.622411,
		0.614155, 0.605889, 0.597615, 0.589338, 0.581059, 0.572781, 0.564508, 0.556242, 0.547987, 0.539744,
		0.531516, 0.523307, 0.515118, 0.506953, 0.498814, 0.490702, 0.482622, 0.474575, 0.466564, 0.458590,
		0.450657, 0.442766, 0.434919, 0.427120, 0.419369, 0.411668, 0.404021, 0.396428, 0.388891, 0.381413,
		0.373994, 0.366637, 0.359344, 0.352115, 0.344953, 0.337858, 0.330833, 0.323879, 0.316996, 0.310186,
		0.303451, 0.296790, 0.290207, 0.283700, 0.277272, 0.270923, 0.264655, 0.258467, 0.252361, 0.246337,
		0.240396, 0.234538, 0.228764, 0.223075, 0.217470, 0.211950, 0.206516, 0.201168, 0.195905, 0.190728,
		0.185637, 0.180633, 0.175714, 0.170882, 0.166136, 0.161475, 0.156900, 0.152411, 0.148007, 0.143687,
		0.139453, 0.135302, 0.131235, 0.127252, 0.123351, 0.119533, 0.115796, 0.112141, 0.108566, 0.105071,
		0.101655, 0.098317, 0.095057, 0.091875, 0.088768, 0.085737, 0.082780, 0.079897, 0.077087, 0.074349,
		0.071682, 0.069085, 0.066557, 0.064097, 0.061704, 0.059378, 0.057117, 0.054920, 0.052786, 0.050714,
		0.048703, 0.046752, 0.044861, 0.043027, 0.041250, 0.039528, 0.037862, 0.036249, 0.034688, 0.033179,
		0.031721, 0.030311, 0.028950, 0.027636, 0.026368, 0.025145, 0.023966, 0.022830, 0.021736, 0.020682,
		0.019669, 0.018694, 0.017757, 0.016856, 0.015991, 0.015161, 0.014365, 0.013601, 0.012870, 0.012169,
		0.011498, 0.010856, 0.010242, 0.009656, 0.009096, 0.008561, 0.008051, 0.007565, 0.007102, 0.006661,
		0.006241, 0.005842, 0.005463, 0.005103, 0.004761, 0.004437, 0.004130, 0.003839, 0.003565, 0.003305,
		0.003059, 0.002827, 0.002609, 0.002403, 0.002210, 0.002028, 0.001857, 0.001696, 0.001546, 0.001405,
		0.001274, 0.001151, 0.001037, 0.000931, 0.000832, 0.000741, 0.000656, 0.000579, 0.000507, 0.000442,
		0.000383, 0.000329, 0.000281, 0.000238, 0.000199, 0.000166, 0.000138, 0.000114, 0.000094, 0.000079,
		0.000069, 0.000062
};
