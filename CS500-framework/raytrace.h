///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#include <vector>
#include <optional>
#include <chrono>

#include "shape.h"
//#include "acceleration.h"
#define _OPENMP_LLVM_RUNTIME
#include "acceleration.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <glm/gtc/epsilon.hpp>

class Shape;

const float PI = 3.14159f;
const float Radians = PI/180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks;
    float alpha;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0) {}
    Material(const vec3 d, const vec3 s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Camera
struct Camera{
    vec3 eye;      // Position of eye for viewing scene
    quat orient;   // Represents rotation of -Z to view direction
    float ry;
    Camera():eye(vec3(5.55323, 2.94275, 2.90087)), orient(quat(0.279589, 0.480987, 0.718247, 0.416981)),ry(0.2) { }
    Camera(const vec3& _eye, const quat& _o, const float _ry)
    {
        eye = _eye; orient = _o; ry = _ry;
    }
};
////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

class Scene {
public:
    int width, height;
    Realtime* realtime;         // Remove this (realtime stuff)
    Material* currentMat;

    Camera sceneCam;
    AccelerationBvh* aBvh = nullptr;
    std::vector<Shape*> shapeList;
    std::vector<Shape*> lightList;



    const float RUSSIAN_ROULETTE = 0.8f;

    const float EPSILON = 0.001;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    Intersection TraceRay(Ray r);
    Color TracePath(Ray r);

    Intersection SampleLight();
    float PdfLight(const Intersection& L) {
        return 1.0f / (L.shape->Area() * lightList.size());
    }

    float GeometryFactor(const Intersection& A, const Intersection& B);

    bool SamePoint(const Intersection& A, const Intersection& B);



    bool isNan(Color C);
    bool isInf(Color C);

    void TraceImage(Color* image, const int pass);

    void SetCameraData();
};
