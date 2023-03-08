//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#include "AuxilaryFunctions.h"




#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



Scene::Scene() { 
    realtime = new Realtime(); 
}

void Scene::Finit()
{
    aBvh = new AccelerationBvh(shapeList);
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    realtime->triangleMesh(mesh); 

    Shape* s = new Tri(mesh, mesh->mat);
    if (currentMat->isLight()) lightList.push_back(s);
    else                    shapeList.push_back(s);
}

quat Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        realtime->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        realtime->setCamera(vec3(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); 
    
    }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        realtime->setAmbient(vec3(f[1], f[2], f[3])); }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]); }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat); 

        Shape* s = new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        if(currentMat->isLight()) lightList.push_back(s);
        shapeList.push_back(s);
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        realtime->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat); 
    
        Shape* s = new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
        if (currentMat->isLight()) lightList.push_back(s);
        shapeList.push_back(s);
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat); 

        Shape* s = new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
        if (currentMat->isLight()) lightList.push_back(s);
        shapeList.push_back(s);
    }



    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    else if (c == "union") {
        Shape* obj2 = shapeList.back();
        shapeList.pop_back();
        Shape* obj1 = shapeList.back();
        shapeList.pop_back();
        shapeList.push_back(new Union(obj1, obj2, currentMat));
    }
    else if (c == "intersect") {
        Shape* obj2 = shapeList.back();
        shapeList.pop_back();
        Shape* obj1 = shapeList.back();
        shapeList.pop_back();
        shapeList.push_back(new Intersect(obj1, obj2, currentMat));
    }
    else if (c == "difference") {
        Shape* obj2 = shapeList.back();
        shapeList.pop_back();
        Shape* obj1 = shapeList.back();
        shapeList.pop_back();
        shapeList.push_back(new Difference(obj1, obj2, currentMat));
    }

    else if (c == "torus") {
        // syntax: torus x y z R r

        realtime->cylinder(vec3(f[1], f[2], f[3]-f[5]), vec3(0,0, 2*f[5]), f[4]+f[5], currentMat);
        realtime->cylinder(vec3(f[1], f[2], f[3]-f[5]), vec3(0,0, 2*f[5]), f[4]-f[5], currentMat);
        Shape* s = new Torus(vec3(f[1], f[2], f[3]), f[4], f[5], currentMat);
        if (currentMat->isLight()) lightList.push_back(s);
        shapeList.push_back(s);
    }

    else if (c == "cone") {
    // syntax: cone x y z h theta

    realtime->cylinder(vec3(f[1], f[2], f[3]+ f[4]/2), vec3(0, 0, f[4]), f[5], currentMat);
    Shape* s = new Cone(vec3(f[1], f[2], f[3]),f[4], f[5], currentMat);
    if (currentMat->isLight()) lightList.push_back(s);
    shapeList.push_back(s);
    }


    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

Intersection Scene::TraceRay(Ray r) {
    return aBvh->intersect(r);
}

glm::vec3 maxF = glm::vec3(0, 0, 0);
Color Scene::TracePath(Ray r) {
    Color C = Color(0, 0, 0);// Accumulated light
    glm::vec3 W(1, 1, 1);// Accumulated weight

    // Initial Ray
    Intersection P = TraceRay(r);
    glm::vec3 N = P.n;
    if (P.miss) return C;
    if (P.shape->material->isLight()) return P.shape->EvalRadiance();

    glm::vec3 omegaO = glm::normalize( - r.d);

    
    while (AuxilaryFunctions::random() <= RUSSIAN_ROULETTE) {
        N = glm::normalize(P.n);

        // Explicit light connection
        Intersection L = SampleLight();
        float p = PdfLight(L) / GeometryFactor(P,L);
        glm::vec3 omegaI = glm::normalize(L.point - P.point);


        Intersection I = TraceRay(Ray(P.point, omegaI));

        if (p > 0 && !I.miss && SamePoint(L,I)) {
            const glm::vec3 f = P.shape->EvalScattering(omegaO, N, omegaI);
            C += 0.5f * W * f/p * L.shape->EvalRadiance();
        }

        // Extend path
        omegaI = glm::normalize(P.shape->SampleBrdf(omegaO, N));
        Intersection Q = TraceRay(Ray(P.point, omegaI));

        if (Q.miss) break; 

        const glm::vec3 f = P.shape->EvalScattering(omegaO, N, omegaI);
        p = P.shape->PdfBrdf(omegaO, N, omegaI) * RUSSIAN_ROULETTE;

        if (p < EPSILON) break;

        if (maxF.x < f.x) {
            maxF = f;
            std::cout << "f: " << glm::to_string(f) << " p: " << p << " f/p: " << glm::to_string(f / p) << std::endl;
        }
        
        W *= f / p;

        if (Q.shape->material->isLight()) {
            C += 0.5f * W * Q.shape->EvalRadiance();
            break;
        }
        P = Q;
        omegaO = -omegaI;
    }

    return C;
}

Intersection Scene::SampleLight() {
    int index = (int)AuxilaryFunctions::random(0, lightList.size()-0.1f);

    Sphere* lightChoice = dynamic_cast<Sphere*>(lightList[index]);

    if (!lightChoice) {
        std::cerr << "ERROR: light not a sphere!!!" << std::endl;
        return Intersection();
    }

    return lightChoice->SampleSphere();
}

float Scene::GeometryFactor(const Intersection& A, const Intersection& B) {
    const glm::vec3 D = A.point - B.point;
    return abs( dot(A.n, D) * dot(B.n, D) / pow(dot(D, D), 2) );
}

bool Scene::SamePoint(const Intersection& A, const Intersection& B) {
    glm::vec3 diff = A.point - B.point;
    return A.shape == B.shape && glm::length2(diff) < EPSILON;//&& A.point == B.point;
}


bool Scene::isNan(Color C) {
    return std::isnan(C.r) || std::isnan(C.g) || std::isnan(C.b);
}

bool Scene::isInf(Color C) {
    return std::isinf(C.r) || std::isinf(C.g) || std::isinf(C.b);
}

void Scene::TraceImage(Color* image, const int pass){

    const glm::vec3 X = (sceneCam.ry * ((float)width) / height) * transformVector(sceneCam.orient, glm::vec3(1, 0, 0));
    const glm::vec3 Y = sceneCam.ry * transformVector(sceneCam.orient, glm::vec3(0, 1, 0));
    const glm::vec3 Z = transformVector(sceneCam.orient, glm::vec3(0, 0, 1));

    for (int _ = 0; _ < pass; ++_) {
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const glm::vec2 d = glm::vec2(2 * (x + AuxilaryFunctions::random()) / width - 1,
                    2 * (y +AuxilaryFunctions::random()) / height - 1);
                const Ray r = Ray(sceneCam.eye, normalize(d.x * X + d.y * Y - Z));
                Color C = TracePath(r);
                
                if (!isNan(C) && !isInf(C)) {
                    image[y * width + x] += C;
                }
                else if (isInf(C)) {
                    std::cout << "infy: " << glm::to_string(C) << std::endl;
                }
            }
        }
    }
}


void Scene::SetCameraData() {
    realtime->run();

    sceneCam = realtime->getCamera();

    std::cout << "Camera Data:\n"
        << "\teye: (" << sceneCam.eye.x << ", " << sceneCam.eye.y << ", " << sceneCam.eye.z << ")\n"
        << "\torientation: " << sceneCam.orient.x << ", "
        << sceneCam.orient.y << ", "
        << sceneCam.orient.z << ", "
        << sceneCam.orient.w << ")\n"
        << "\try: " << sceneCam.ry << std::endl;
}

