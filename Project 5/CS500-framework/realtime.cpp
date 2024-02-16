////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLFW window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

// Stupid C++ needs callbacks to be static functions.
static Realtime* globalRealtime = nullptr;
void CBKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods)  {
    globalRealtime->Keyboard(window, key, scancode, action, mods); }
void CBMouseButton(GLFWwindow* window, int button, int action, int mods)  {
    globalRealtime->MouseButton(window, button, action, mods); } 
void CBMouseMotion(GLFWwindow* window, double x, double y)  {
    globalRealtime->MouseMotion(window, x, y); }
void CBScroll(GLFWwindow* window, double x, double y)  {
    globalRealtime->Scroll(window, x, y); }

//void CBAnimate(int value)
//{
//    glutTimerFunc(30, CBAnimate, 1);
//    // atime = 360.0*glutGet(GLUT_ELAPSED_TIME)/12000;
//    glutPostRedisplay();
//}

unsigned int MakeVAO(MeshData* meshdata)
{
    unsigned int vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    
    if (meshdata && meshdata->vertices.size() == 0) {
        std::cerr << "Missing meshdata->vertices in MakeVAO\n";
        exit(-1); }
    if (meshdata && meshdata->triangles.size() == 0) {
        std::cerr << "Missing meshdata->triangles in MakeVAO\n";
        exit(-1); }
    
    GLuint Pbuff;
    glGenBuffers(1, &Pbuff);
    glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*11*meshdata->vertices.size(), 
                 &(meshdata->vertices[0].pnt[0]), GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)0);
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(3*sizeof(float)));
    
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(6*sizeof(float)));
    
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(8*sizeof(float)));
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    
    GLuint Ibuff;
    glGenBuffers(1, &Ibuff);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*3*meshdata->triangles.size(),
                 &(meshdata->triangles[0][0]), GL_STATIC_DRAW);
    
    glBindVertexArray(0);

    return vao;
}

MeshData* SphMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n*2;  i++) {
        float s = i*2.0f*PI/float(n*2);
        for (unsigned int j=0;  j<=n;  j++) {
            float t = j*PI/float(n);
            float x = cos(s)*sin(t);
            float y = sin(s)*sin(t);
            float z = cos(t);
            meshdata->vertices.push_back(VertexData(vec3(x,y,z),
                                                    vec3(x,y,z),
                                                    vec2(s/(2*PI), t/PI),
                                                    vec3(sin(s), cos(s), 0.0)));
            if (i>0 && j>0) {
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1), 
                                                      (i-1)*(n+1) + (j  ), 
                                                      (i  )*(n+1) + (j  )));
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1),
                                                      (i  )*(n+1) + (j  ),
                                                      (i  )*(n+1) + (j-1))); } } }
    return meshdata;
}

MeshData* BoxMesh()
{
    mat4 face[6] = {
        Identity(),
        rotate(180.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(0.0f, 1.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(0.0f, 1.0f, 0.0f))};
       
    mat4 half = translate(vec3(0.5f, 0.5f, 0.5f))*scale(vec3(0.5f, 0.5f, 0.5f));
    MeshData* meshdata = new MeshData();
    for (unsigned int f=0;  f<6;  f++) {
        mat4 m4 = half*face[f];
        mat3 m3 = mat3(m4); // Extracts 3x3 from a 4x4
        for (unsigned int i=0;  i<2;  i++) {
            for (unsigned int j=0;  j<2;  j++) {
              vec4 p = m4*vec4(float(2*i)-1.0f, float(2*j)-1.0f, 1.0f, 1.0f);
              vec3 tnrm = m3*vec3(0.0f, 0.0f, 1.0f);
              vec3 ttan = m3*vec3(1.0, 0.0, 0.0);
              meshdata->vertices.push_back(VertexData(vec3(p[0], p[1], p[2]),
                                                      vec3(tnrm[0], tnrm[1], tnrm[2]),
                                                      vec2(float(i), float(j)),
                                                      vec3(ttan[0], ttan[1], ttan[2])));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+1, 4*f+3));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+3, 4*f+2)); } } }
    return meshdata;
}

MeshData* CylMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n;  i++) {
        float s = i*2.0f*PI/float(n);
        float x = cos(s);
        float y = sin(s);
        
        meshdata->vertices.push_back(VertexData(vec3(x, y, 0.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        meshdata->vertices.push_back(VertexData(vec3(x, y, 1.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        if (i>0) {
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i-1)*2, (i  )*2));
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i  )*2, (i  )*2+1)); } }
    return meshdata;
}

////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates a OpenGL Shader.
////////////////////////////////////////////////////////////////////////
void ShaderProgram::CreateShader(const std::string fname, const GLenum type)
{
    // Read a file into a string
    std::ifstream f;
    f.open(fname, std::ios_base::binary); // Open
    f.seekg(0, std::ios_base::end);       // Position at end
    int length = f.tellg();               // to get the length
    
    char* src = new char [length+1];  // Create buffer of needed length
    f.seekg (0, std::ios_base::beg);      // Position at beginning
    f.read (src, length);             //   to read complete file
    f.close();                            // Close
    
    src[length] = char(0);            // Finish with a NULL
    
    // Create a shader, attach, send it the source, and compile it.
    int shader = glCreateShader(type);
    glAttachShader(program, shader);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    
    // Get the compilation status
    int status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    
    // If compilation status is not OK, get and print the log message.
    if (status != 1) {
        int length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        char* buffer = new char[length];
        glGetShaderInfoLog(shader, length, nullptr, buffer);
        printf("Compile log(%s):\n%s\n", type==GL_VERTEX_SHADER?"Vertex":"Fragment", buffer);
        delete buffer;
    }
}

void ShaderProgram::LinkProgram()
{
    // Link program and check the status
    glLinkProgram(program);
    int status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    
    // If link failed, get and print log
    if (status != 1) {
        int length;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        char* buffer = new char[length];
        glGetProgramInfoLog(program, length, nullptr, buffer);
        printf("Link log:\n%s\n", buffer);
        delete buffer;
    }
}


void applyMaterial(Material* mat, const unsigned int program)
{
    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);
    
    loc = glGetUniformLocation(program, "Ks");
    glUniform3fv(loc, 1, &mat->Ks[0]);
    
    loc = glGetUniformLocation(program, "alpha");
    glUniform1f(loc, mat->alpha);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mat->texid);
    loc = glGetUniformLocation(program, "tex");
    glUniform1i(loc, 0);
    
    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 0);
}

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
void applyLight(Material* mat, const unsigned int program)
{
    vec3 Z;
    
    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);
    
    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 1);
}

////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
Obj::Obj(MeshData* m, const mat4& tr, Material* b)
    : meshdata(m), modelTR(tr), material(b)
{
    vec4 sum(0,0,0,0);
    //for (int i=0;  i<meshdata->vertices.size();  i++)
    //    sum += modelTR*vec4(v.pnt[0], v.pnt[1], v.pnt[2], 1.0);
    for (auto v : meshdata->vertices) // C++11 for loop
        sum += modelTR*vec4(v.pnt[0], v.pnt[1], v.pnt[2], 1.0);

    vec4 ave = sum/float(meshdata->vertices.size());
    center = vec3(ave[0], ave[1], ave[2]);
    
    //std::cout << "center: " << center.transpose() << std::endl;
    
    vao = MakeVAO(meshdata);
}

void Obj::draw()
{
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 3*meshdata->triangles.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////

static void error_callback(int error, const char* msg)
{
    fputs(msg, stderr);
}

// Constructor for Realtime.  Initializes OpenGL, and GLFW, as well as
// the data elements of the class.
Realtime::Realtime()
{   
    glfwSetErrorCallback(error_callback);
    // Initialize the OpenGL bindings
    glbinding::Binding::initialize(false);

    globalRealtime = this;
    // Initialize GLFW
    if (!glfwInit())  exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_RESIZABLE, 1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, 0);

    window = glfwCreateWindow(200, 200, "CS500 Framework", NULL, NULL);
    if (!window)  { glfwTerminate();  exit(-1); }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, CBKeyboard);
    glfwSetMouseButtonCallback(window, CBMouseButton);
    glfwSetCursorPosCallback(window, CBMouseMotion);
    glfwSetScrollCallback(window, CBScroll);


    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Rendered by: %s\n", glGetString(GL_RENDERER));
    fflush(stdout);
    
    // Create the shader program
    lighting.CreateProgram();
    lighting.CreateShader("realtime.vert", GL_VERTEX_SHADER);
    lighting.CreateShader("realtime.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lighting.program, 0, "vertex");
    glBindAttribLocation(lighting.program, 1, "vertexNormal");
    glBindAttribLocation(lighting.program, 2, "vertexTexture");
    glBindAttribLocation(lighting.program, 3, "vertexTangent");
    lighting.LinkProgram();

    // Several generic meshes which can be transfofrmed to *any* sphere, box, or cylinder.
    sphMesh = SphMesh();
    boxMesh = BoxMesh();
    cylMesh = CylMesh();

    // Initialize various member attributes
    //materials.push_back(new Material());

    nav = false;
    spin = 0.0f;
    tilt = 90.0f;
    speed = 0.05f;
    front = 0.1f;
    back = 1000.0f;

    shifted = false;
    leftDown = false;
    middleDown = false;
    rightDown = false;
    motionkey = 0;

    ambient = vec3(0.2, 0.2, 0.2); 
}

// This function enters the event loop.
void Realtime::run()
{
    for (int i=0;  i<lights.size();  i++) {
        lightPosn[i] = lights[i]->Center();
        float d = length(lightPosn[i]);
        lightEmit[i] = lights[i]->material->Kd*lights[i]->area/(d*d)/3.14;
        printf("light %d: (%f %f %f) (%f %f %f) %f\n", i,
               lightEmit[i][0], lightEmit[i][1], lightEmit[i][2],
               lightPosn[i][0], lightPosn[i][1], lightPosn[i][2],
               lights[i]->area); }
    
    cDist = length(eye);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        Realtime::DrawScene();
        glfwSwapBuffers(window); }

    glfwTerminate();
}

// Called when the scene needs to be redrawn.
void Realtime::DrawScene()
{
    // Get the window's current size
    glfwGetWindowSize(window, &width, &height);
    glViewport(0, 0, width, height);
    
    vec3 viewDir = ViewDirection();
    vec2 dir2 = normalize(vec2(viewDir[0], viewDir[1]));
    if (motionkey == GLFW_KEY_W)
        eye += speed*vec3(dir2[0], dir2[1], 0.0);
    if (motionkey == GLFW_KEY_S)
        eye -= speed*vec3(dir2[0], dir2[1], 0.0);
    if (motionkey == GLFW_KEY_D)
        eye += speed*vec3(dir2[1], -dir2[0], 0.0);
    if (motionkey == GLFW_KEY_A)
        eye -= speed*vec3(dir2[1], -dir2[0], 0.0);
    if (motionkey == GLFW_KEY_SPACE)
        eye -= speed*vec3(0.0f, 0.0f, -1.0f);
    if (motionkey == GLFW_KEY_Z)
        eye -= speed*vec3(0.0f, 0.0f, 1.0f);
    
    int loc;

    glClearColor(0.3,0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

    mat4 WorldView;
    mat4 R = toMat4(conjugate(ViewQuaternion()));
    WorldView = R*translate(-eye);

    float rx = (ry*width)/height;
    mat4 WorldProj = frustum(-front*rx, front*rx, -front*ry, front*ry, front, back);

    lighting.Use();

    loc = glGetUniformLocation(lighting.program, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));

    loc = glGetUniformLocation(lighting.program, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    
    loc = glGetUniformLocation(lighting.program, "ambient");
    glUniform3fv(loc, 1, &ambient[0]);

    loc = glGetUniformLocation(lighting.program, "eyePos");
    glUniform3fv(loc, 1, &eye[0]);

    loc = glGetUniformLocation(lighting.program, "lightNum");
    glUniform1i(loc, lights.size());

    loc = glGetUniformLocation(lighting.program, "lightPosn");
    glUniform3fv(loc, 8, &lightPosn[0][0]);

    loc = glGetUniformLocation(lighting.program, "lightEmit");
    glUniform3fv(loc, 8, &lightEmit[0][0]);
        
    // for (unsigned int i=0;  i<objs.size();  i++) {
    //     Material* material = objs[i]->material;
    //     mat4& modelTR = objs[i]->modelTR;
    //     ...
    //     objs[i]->draw();
    for (auto obj : objs) {     // C++11 loop
        Material* material = obj->material;
        mat4& modelTR = obj->modelTR;
        mat3 normalTR = inverse(mat3(modelTR));

        loc = glGetUniformLocation(lighting.program, "ModelTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(modelTR));

        loc = glGetUniformLocation(lighting.program, "normalTR");
        glUniformMatrix3fv(loc, 1, GL_FALSE, Pntr(normalTR));

        if (!material) {
            std::cerr << "No material associated with object\n";
            exit(-1); }
        if (material->isLight())
            applyLight(material, lighting.program);
        else
            applyMaterial(material, lighting.program);
        obj->draw(); }

    lighting.Unuse();
}

// Called by GLFW for keyboard actions.
void Realtime::Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_REPEAT) return; // Because keyboard autorepeat is evil.
     
    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)
        shifted = !shifted;

    if  (action == GLFW_PRESS) {
        switch(key) {
        case GLFW_KEY_TAB:
        nav = !nav;
        break;

        case GLFW_KEY_V: {
            quat q = ViewQuaternion();
            printf("camera  %g %g %g   %g   q %g %g %g %g\n",
                   eye[0], eye[1], eye[2], ry,  q.w, q.x, q.y, q.z);
            printf("screen %d %d\n", width, height);
            fflush(stdout); }
            break;

        case GLFW_KEY_W: case GLFW_KEY_S: case GLFW_KEY_A: case GLFW_KEY_D:
        case GLFW_KEY_SPACE: case GLFW_KEY_Z:
            motionkey = key;
            break;
        
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            glfwSetWindowShouldClose(window, true);
            break;
        } }

    else //action == GLFW_RELEASE
    motionkey = 0;
}

// Called by GLFW when a mouse button changes state.
void Realtime::MouseButton(GLFWwindow* window, int button, int action, int mods)
{        
    // Record the position of the mouse click.
    glfwGetCursorPos(window, &mouseX, &mouseY);
    
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftDown = (action == GLFW_PRESS); }

    else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        middleDown = (action == GLFW_PRESS);  }

    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        rightDown = (action == GLFW_PRESS); }
}

void Realtime::MouseMotion(GLFWwindow* window, double x, double y)
{
    // Calculate the change in the mouse position
    double dx = x-mouseX;
    double dy = y-mouseY;

    if (leftDown) {        // Rotate light position
        if (nav) {
            spin += dx/2.0f;
            tilt += dy/2.0f; }
        else {
            vec3 C = eye + cDist*ViewDirection();
            spin += dx/2.0f;
            tilt += dy/2.0f;
            eye = C - cDist*ViewDirection(); } }

    else if (middleDown) { }

    // Record this position
    mouseX = x;
    mouseY = y;
}

void Realtime::Scroll(GLFWwindow* window, double x, double y)
{
    if (y>0.0) {
        vec3 C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.0f/1.02f); 
        eye = C - cDist*ViewDirection(); }

    else if (y<0.0) {
        vec3 C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.02f);
        eye = C - cDist*ViewDirection(); }
}


void Realtime::sphere(const vec3 center, const float r, Material* mat)
{
    mat4 m = translate(center) * scale(vec3(r,r,r));
    vec3 rrr(r,r,r);
    Obj* obj = new Obj(sphMesh, m, mat);
    obj->area = 4*PI*r*r;
    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}

void Realtime::box(const vec3 base, const vec3 diag, Material* mat)
{
    mat4 m = translate(base) * scale(vec3(diag[0],diag[1],diag[2]));
    Obj* obj = new Obj(boxMesh, m, mat);
    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}


void Realtime::cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat)
{
    vec3 Z(0.0f, 0.0f, 1.0f);
    vec3 C = normalize(axis);
    vec3 B = cross(C,Z);
    if (length(B) <1e-8)
        B = vec3(0,1,0);
    else
        B = normalize(B);
    vec3 A = normalize(cross(B,C));

    mat4 R(A[0], A[1], A[2], 0.0f,
           B[0], B[1], B[2], 0.0f,
           C[0], C[1], C[2], 0.0f,
           0.0f, 0.0f, 0.0f, 1.0f);           

    mat4 m = translate(base)*R*scale(vec3(radius, radius, length(axis)));
    vec3 rrr(radius,radius,radius);
    Obj* obj = new Obj(cylMesh, m, mat);
    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}
