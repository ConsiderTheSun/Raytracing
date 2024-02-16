///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
#endif

#include "geom.h"
#include "raytrace.h"

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene){

    std::ifstream input(inName.c_str());
    if (input.fail()) {
        std::cerr << "File not found: "  << inName << std::endl;
        fflush(stderr);
        exit(-1); 
    }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        std::vector<std::string> strings;
        std::vector<float> floats;
        
        // Parse as parallel lists of strings and floats
        std::stringstream lineStream(line);
        for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
            float f;
            //std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
            if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
            floats.push_back(f);
            strings.push_back(s); }

        if (strings.size() == 0) continue; // Skip blanks lines
        if (strings[0][0] == '#') continue; // Skip comment lines
        
        // Pass the line's data to Command(...)
        scene->Command(strings, floats);
    }

    input.close();
}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image, int pass)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x];
            *dp++ = pixel[0]/pass;
            *dp++ = pixel[1] / pass;
            *dp++ = pixel[2] / pass; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete data;
}


////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    //std::string default = "testsceneReflection.scn";
    //std::string default = "sdfTransform.scn";
    //std::string default = "sdfClassic.scn";
    //std::string default = "sdfExtra.scn";
    //std::string default = "sdfFields.scn";
    //std::string default = "sdfShapes.scn";
    std::string default = "sdfTwist.scn";
    //std::string default = "testsceneTransmission.scn";
    // 
    //std::string default = "sceneA.scn";

    // 
    //std::cout << "scene: ";
    //std::cin >> default;
    

    Scene* scene = new Scene();

    // Read the command line argument
    std::string inName =  (argc > 1) ? argv[1] : default;
    std::string hdrBaseName = inName.substr(0, inName.size() - 4);

    //hdrBaseName.replace(hdrName.size()-3, hdrName.size(), "hdr");


    // Read the scene, calling scene.Command for each line.
    ReadScene(inName, scene);

    scene->Finit();

    // Allocate and clear an image array
    Color *image =  new Color[scene->width*scene->height];
    for (int y=0;  y<scene->height;  y++)
        for (int x=0;  x<scene->width;  x++)
            image[y*scene->width + x] = Color(0,0,0);

    // uses realtime to set the camera data
    scene->SetCameraData();

    //std::array<int, 7> occasionally = { 0, 1, 8, 64, 512, 4096, 8192 };
    std::array<int, 6> occasionally = { 0, 1, 8, 64, 512, 4096 };
    //std::array<int, 5> occasionally = { 0, 1, 8, 64, 512 };
    //std::array<int, 4> occasionally = { 0, 1, 8, 64 };
    //std::array<int, 4> occasionally = { 0, 64, 128, 256 };
    //std::array<int, 3> occasionally = { 0, 1, 8 };

    //std::array<int, 3> occasionally = { 0, 64, 512 };
    //std::array<int, 9> occasionally = { 0, 1, 2, 3, 4, 5, 6, 7 ,8 };
    //std::array<int, 5> occasionally = { 0, 1, 2, 3, 4 };

    std::chrono::time_point<std::chrono::high_resolution_clock> start =
        std::chrono::high_resolution_clock::now();

    for (int i = 1; i < occasionally.size(); i++) {
        // RayTrace the image
        scene->TraceImage(image, occasionally[i]- occasionally[i-1]);

        // output occasionally
        std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        std::cout << "finished " + std::to_string(occasionally[i]) + " with time " << duration.count() << std::endl;

        // Write the image
        WriteHdrImage(hdrBaseName + std::to_string(occasionally[i])+".hdr", scene->width, scene->height, image, occasionally[i]);
    }

}
