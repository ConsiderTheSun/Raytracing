#ifndef SHAPE_H
#define SHAPE_H

#include <limits>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>

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

class Ray {
public:
	vec3 o, d;
	Ray(const vec3 _o, const vec3 _d) : o(_o), d(glm::normalize(_d)) {}
};
class Shape;

class Intersection {
public:
	Intersection() :shape(nullptr),miss(true), t(0),n(glm::vec3()),point(glm::vec3()) {}
	Intersection(Shape* s, float _t, glm::vec3 _n, glm::vec3 _point) :shape(s),miss(false), t(_t), n(_n), point(_point) {}
	Shape* shape;
	bool miss;
	float t;
	glm::vec3 point;
	glm::vec3 n;
	float distance() const { return t; }  // A function the BVH traversal needs to be supplied.
};

class Shape {
public:
	virtual Intersection intersect(Ray r)=0;
	virtual float distance(const glm::vec3& P) const { return 100000; }; // TODO: make pure virtual

	virtual std::vector<glm::vec3> pointList()=0;

	Shape() :material(NULL) {}
	Shape(Material* m) :material(m) {}

	Material * material;

	glm::vec3 SampleBrdf(glm::vec3 omegaO, glm::vec3 N);
	float PdfBrdf(glm::vec3 omegaO, glm::vec3 N, glm::vec3 omegaI);
	glm::vec3 EvalScattering(glm::vec3 omegaO, glm::vec3 N, glm::vec3 omegaI, float attenuationDistance);

	virtual float Area() { std::cout << "Area not implemented for this shape" << std::endl; return 0; }

	float Kai(float d);
	float D(glm::vec3 m, glm::vec3 N);

	float G(glm::vec3 omegaI, glm::vec3 omegaO, glm::vec3 m, glm::vec3 N);
	float G1(glm::vec3 v, glm::vec3 m, glm::vec3 N);

	glm::vec3 F(float d);


	// these functions assume the shape is a light
	Color EvalRadiance();
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
	float distance(const glm::vec3& P) const;

	std::vector<glm::vec3> pointList();

	Intersection SampleSphere();

	float Area() override { return 4 * M_PI * radius * radius; }

private:
	glm::vec3 center;
	float radius;
	
};


class Box : public Shape {
public:
	Box(glm::vec3 c, glm::vec3 d, Material* m) :corner(c), diagonal(d), Shape(m) {}

	Intersection intersect(Ray r);
	float distance(const glm::vec3& P) const;

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


class RayMarchShape : public Shape {
public:
	RayMarchShape(Material* m) :Shape(m) {}
	Intersection intersect(Ray r);
	virtual float distance(const glm::vec3& P) const override = 0;
};


class Union : public RayMarchShape {
	Shape* A;
	Shape* B;
public:
	Union(Shape* _A, Shape* _B, Material* m) : A(_A), B(_B), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override{
		return std::min(A->distance(P), B->distance(P));
	}

	std::vector<glm::vec3> pointList() {
		std::vector<glm::vec3> ABList = A->pointList();
		std::vector<glm::vec3> BList = B->pointList();
		ABList.insert(ABList.end(), BList.begin(), BList.end());
		return ABList;
	}
};

class Intersect : public RayMarchShape {
	Shape* A;
	Shape* B;
public:
	Intersect(Shape* _A, Shape* _B, Material* m) : A(_A), B(_B), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override {
		return std::max(A->distance(P), B->distance(P));
	}

	std::vector<glm::vec3> pointList() {
		std::vector<glm::vec3> ABList = A->pointList();
		std::vector<glm::vec3> BList = B->pointList();
		ABList.insert(ABList.end(), BList.begin(), BList.end());
		return ABList;
	}
};

class Difference : public RayMarchShape {
	Shape* A;
	Shape* B;
public:
	Difference(Shape* _A, Shape* _B, Material* m) : A(_A), B(_B), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override {
		return std::max(A->distance(P), -B->distance(P));
	}

	std::vector<glm::vec3> pointList() {
		std::vector<glm::vec3> ABList = A->pointList();
		std::vector<glm::vec3> BList = B->pointList();
		ABList.insert(ABList.end(), BList.begin(), BList.end());
		return ABList;
	}
};

class Torus : public RayMarchShape{
	glm::vec3 center;
	float R, r;

public:
	Torus(glm::vec3 c, float _R, float _r, Material* m) : center(c), R(_R), r(_r), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override {
		glm::vec3 p = P - center;
		float m = sqrt(p.x * p.x + p.y * p.y) - R;
		return sqrt(m * m + p.z * p.z) - r;
	}
	std::vector<glm::vec3> pointList();

};

class Cone : public RayMarchShape {
	glm::vec3 center;
	float height, theta;

public:
	Cone(glm::vec3 c, float h, float _theta, Material* m) : center(c), height(h),theta(_theta), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

#endif