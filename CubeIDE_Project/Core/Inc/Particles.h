/*
 * particles.h
 *
 *  Created on: 7 May 2021
 *      Author: EA
 */

#ifndef INC_PARTICLES_H_
#define INC_PARTICLES_H_

class vec2
{
public:
	float x;
	float y;
//	float length()
//	{
//		return sqrt(x*x+y*y);
//	}

};

class Particle
{
public:
	Particle();
	virtual ~Particle();
//	Rotate();
//	Update();

private:
	struct ParticleProps
	{
		vec2 Position;
		vec2 Velocity;
		vec2 Accelartion;
		float SizeBegin;
		float SizeEnd;
		float SizeVariation;
		float LifeTime = 1.0f;
	};
};

//class Appearance
// {
//public:
//	Appearance();
//
//private:
//	struct ParticleProps
//	{
//		vec2 Position;
//		vec2 Velocity;
//		vec2 VelocityVariation;
////	glm::vec4 ColorBegin, ColorEnd;
//		float SizeBegin;
//		float SizeEnd;
//		float SizeVariation;
//		float LifeTime = 1.0f;
//	};
//};

//particle

#endif /* INC_PARTICLES_H_ */
