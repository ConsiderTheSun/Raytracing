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




#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() 
{ 
    realtime = new Realtime(); 
}

void Scene::Finit()
{

    aBvh = new AccelerationBvh(shapeList);
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    /*if (mesh->triangles.size()>500) {
        std::vector<ivec3> newTris;
        for (int i = 0; i < mesh->triangles.size() / 4; ++i) {
            newTris.push_back(mesh->triangles[i]);
        }
        mesh->triangles = newTris;

        ivec3 data(mesh->triangles[100]);
        mesh->triangles.resize(1);
        mesh->triangles[0] = data;
    }
    else {
        return;
    }*/

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
        else                    shapeList.push_back(s);
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        realtime->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat); 
    
        Shape* s = new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
        if (currentMat->isLight()) lightList.push_back(s);
        else                    shapeList.push_back(s);
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat); 

        Shape* s = new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
        if (currentMat->isLight()) lightList.push_back(s);
        else                    shapeList.push_back(s);
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

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

Intersection Scene::TraceRay(Ray r) {
    return aBvh->intersect(r);

    /*
    // The old method without the spatial data structure speedup
    Intersection closestIntersection = Intersection();

    if (shapeList.size() == 0) return closestIntersection;
    
    for (Shape* shape : shapeList) {
        Intersection newIntersection = shape->intersect(r);

        if (!newIntersection.miss && newIntersection.t>0 && (newIntersection.t < closestIntersection.t|| closestIntersection.miss)) {
            closestIntersection = newIntersection;
        }
    }
    return closestIntersection;*/
}

void Scene::TraceImage(Color* image, const int pass)
{
    realtime->run();

    sceneCam = realtime->getCamera();

    std::cout << "Camera Data:\n"
        << "\teye: (" << sceneCam.eye.x<<", " << sceneCam.eye.y<<", " << sceneCam.eye.z<<")\n"
        << "\torientation: " << sceneCam.orient.x << ", " 
                             << sceneCam.orient.y << ", "
                             << sceneCam.orient.z << ", "
                             << sceneCam.orient.w << ")\n"
        << "\try: " << sceneCam.ry << std::endl;

    // TEMP LIGHT DATA
    Sphere* lightSphere = dynamic_cast<Sphere*>(lightList[0]);
    if (!lightSphere) {
        std::cerr << "LIGHT SPHERE NOT WORKING!!!" << std::endl;
    }
    const glm::vec3 X = (sceneCam.ry * ((float)width) / height) * transformVector(sceneCam.orient, glm::vec3(1, 0, 0));
    const glm::vec3 Y = sceneCam.ry * transformVector(sceneCam.orient, glm::vec3(0, 1, 0));
    const glm::vec3 Z = transformVector(sceneCam.orient, glm::vec3(0, 0, 1));


    start = std::chrono::high_resolution_clock::now();


#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Color color;

            const glm::vec2 d = glm::vec2(2 * (x + 0.5) / width - 1,
                                          2 * (y + 0.5) / height - 1);
            const Ray r = Ray(sceneCam.eye,normalize(d.x * X + d.y * Y - Z));
            const Intersection i = TraceRay(r);

            if (i.miss) {
                color = Color(1.0, 1.0, 1.0);
            }
            else {
                //color = dot(i.n, lightSphere->getCenter()) * i.shape->material->Kd; // aprox lighting v1
                color = 0.5f* dot(i.n, lightSphere->getCenter()) * i.shape->material->Kd; // aprox lighting
                //color = i.shape->material->Kd; // just kd
                //color = vec3((i.t-5)/4); // t values
                //color = glm::abs(i.n); // normals
            }

            image[y * width + x] = color;
        }
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);

    std::cout << "finished with time " << duration.count() << std::endl;
}
