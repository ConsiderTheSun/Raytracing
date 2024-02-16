#pragma once
#include "geom.h"

struct AuxilaryFunctions {
	static glm::vec3 SampleLobe(glm::vec3 A, float c, float phi);

	static float random(float min = 0, float max = 1);

	static float sign(float x) {
		return (x >= 0) ? 1 : -1;
	}
};