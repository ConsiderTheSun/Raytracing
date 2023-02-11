#pragma once
#include "geom.h"
//#include "shape.h"

struct AuxilaryFunctions {
	static glm::vec3 SampleLobe(glm::vec3 A, float c, float phi);

	static float random(float min = 0, float max = 1) {
		return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min))) + min;
	}

};