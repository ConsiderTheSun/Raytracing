#include "AuxilaryFunctions.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

glm::vec3 AuxilaryFunctions::SampleLobe(glm::vec3 A, float c, float phi) {
    const float s = sqrt(1 - pow(c, 2));

    // Create vector K centered around Z-axis and rotate to A-axis
    const glm::vec3 K = glm::vec3(s * cos(phi), s * sin(phi), c);

    if (abs(A.z - 1) < 0.001) return K; // A==Z so no rotation
    if (abs(A.z + 1) < 0.001) return glm::vec3(K.x, -K.y, -K.z); // A==-Z so rotate 180 around X axis

    if (dot(A, A) > 1.001 || dot(A, A) <  0.999) {
        std::cout << "WARNING: A NOT NORMAL!!!" << std::endl;
        A = normalize(A);
    }

    const glm::vec3 B = normalize(glm::vec3(-A.y, A.x, 0)); // Z x A
    const glm::vec3 C = cross(A, B);
    return K.x * B + K.y * C + K.z * A;
}


float AuxilaryFunctions::random(float min, float max) {
    return (max - min) * myrandom(RNGen) + min;
    //return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min))) + min;
}