#ifndef SHAPE_H
#define SHAPE_H

#include <limits>
#include <algorithm>

class Material;
class MeshData;
class VertexData
{
public:
	vec3 pnt;
	vec3 nrm;
	vec2 tex;
	vec3 tan;
	VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a)
		: pnt(p), nrm(n), tex(t), tan(a)
	{}
};

struct MeshData
{
	std::vector<VertexData> vertices;
	std::vector<ivec3> triangles;
	Material* mat;
};

// FIX THIS:  Dummy ray to allow for compilation
class Ray {
public:
	vec3 o, d;
	Ray(const vec3 _o, const vec3 _d) : o(_o), d(_d) {}
};
class Shape;
// FIX THIS:  This dummy Intersection record is defficient -- just barely enough to compile.
class Intersection {
public:
	Intersection() :shape(nullptr),miss(true), t(0),n(glm::vec3()) {}
	Intersection(Shape* s, float _t, glm::vec3 _n) :shape(s),miss(false), t(_t), n(_n) {}
	Shape* shape;
	bool miss;
	float t;
	glm::vec3 n;
	float distance() const { return t; }  // A function the BVH traversal needs to be supplied.
};

class Shape {
public:
	virtual Intersection intersect(Ray r)=0;
	virtual std::vector<glm::vec3> pointList()=0;

	Shape() :material(NULL) {}
	Shape(Material* m) :material(m) {}

	Material * material;
};

struct Interval {
	Interval(float _t0=0, float _t1= std::numeric_limits<float>::infinity(), 
		glm::vec3 _n0=glm::vec3(), glm::vec3 _n1=glm::vec3()) : t0(_t0), t1(_t1), n0(_n0), n1(_n1) {}
	float t0;
	float t1;
	glm::vec3 n0;
	glm::vec3 n1;
};

struct Slab {
	Slab(glm::vec3 _n, float _d0, float _d1) : n(_n), d0(_d0), d1(_d1) {}
	Interval intersect(Ray r);

	glm::vec3 n;
	float d0;
	float d1;
};

class Sphere : public Shape {
public:
	Sphere(glm::vec3 c, float r, Material* m):center(c),radius(r), Shape(m) {}

	glm::vec3 getCenter() { return center; }

	Intersection intersect(Ray r);
	std::vector<glm::vec3> pointList();

private:
	glm::vec3 center;
	float radius;
	
};


class Box : public Shape {
public:
	Box(glm::vec3 c, glm::vec3 d, Material* m) :corner(c), diagonal(d), Shape(m) {}

	Intersection intersect(Ray r);
	std::vector<glm::vec3> pointList();
private:
	glm::vec3 corner;
	glm::vec3 diagonal;
};

class Cylinder : public Shape {
public:
	Cylinder(glm::vec3 b, glm::vec3 a, float r, Material* m) :base(b), axis(a),radius(r), Shape(m) {}

	Intersection intersect(Ray r);
	std::vector<glm::vec3> pointList();
private:
	glm::vec3 base;
	glm::vec3 axis;
	float radius;
};


class Tri : public Shape {
public:
	Tri(MeshData* md, Material* m) :mesh(md), Shape(m) {}

	Intersection intersect(Ray r);
	std::vector<glm::vec3> pointList();
private:
	MeshData* mesh;
};

#endif