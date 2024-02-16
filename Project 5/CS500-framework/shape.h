/*
 * Credit to Inigo Quilez for a lot of the distance functions in here
 * https://iquilezles.org/articles/distfunctions/
 */


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
	virtual float distance(const glm::vec3& P) const =0; // TODO: make pure virtual

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

	glm::vec3 getCenter() const { return center; }
	float getRadius() const { return radius; }

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
	float distance(const glm::vec3& P) const;

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
	float distance(const glm::vec3& P) const { return 1000000; }; // TODO: make this work if I want mesh

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
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();

};

class Cone : public RayMarchShape {
	glm::vec3 center;
	float height, radius;
	float theta;

public:
	Cone(glm::vec3 c, float _height, float _radius, Material* m) : center(c), height(_height), radius(_radius), theta(sin(_radius / _height)),RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Pyramid : public RayMarchShape {
	float height;

public:
	Pyramid(float _height, Material* m) : height(_height), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Octohedron : public RayMarchShape {
	float s;

public:
	Octohedron(float _s, Material* m) : s(_s), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class TriPrism : public RayMarchShape {
	float side;
	float height;

public:
	TriPrism(float _side, float _height, Material* m) :side(_side), height(_height), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class HexPrism : public RayMarchShape {
	float side;
	float height;

public:
	HexPrism(float _side, float _height, Material* m) :side(_side), height(_height), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Link : public RayMarchShape {
	float le, r1, r2;

public:
	Link(float _le, float _r1, float _r2, Material* m) :le(_le), r1(_r1), r2(_r2), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};


class Translate : public RayMarchShape {
	Shape* A;

	glm::vec3 position;
public:
	Translate(Shape* _A, vec3 _position, Material* m) : A(_A), position(_position), RayMarchShape(m) {}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Rotate : public RayMarchShape {
	Shape* A;

	glm::mat4 R;
	glm::mat4 Rinv;
public:
	Rotate(Shape* _A, vec3 rotationV, Material* m) : A(_A), RayMarchShape(m) {
		rotationV *= M_PI / 180.0f;
		R = glm::rotate(glm::mat4(1.0f), rotationV.x, glm::vec3(1, 0, 0));
		R = glm::rotate(R, rotationV.y, glm::vec3(0, 1, 0));
		R = glm::rotate(R, rotationV.z, glm::vec3(0, 0, 1));

		Rinv = glm::transpose(R);
	}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Scale : public RayMarchShape {
	Shape* A;

	float lConst;
	glm::vec3 scale;
public:
	Scale(Shape* _A, vec3 _scale, Material* m) : A(_A), scale(_scale), RayMarchShape(m) {
		lConst = std::min(std::min(scale.x, scale.y), scale.z);
	}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Twist : public RayMarchShape {
	Shape* A;

	float lConst;
	float twistSpeed;
public:
	Twist(Shape* _A, float _twistSpeed, Material* m) : A(_A), twistSpeed(2.0f * M_PI*_twistSpeed), RayMarchShape(m) {
		lConst = 1.0f/sqrt(4 + pow(_twistSpeed * M_PI, 2));
	}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class MetaBalls : public RayMarchShape {

public:
	struct MetaBallData {
		vec3 position;
		float radius;
		MetaBallData(vec3 p, float r) : position(p), radius(r) {};
	};
	float threshold;
	std::vector<Sphere> ballList;

	float lConst;
	MetaBalls(float _threshold, std::vector<Sphere> _ballList, Material* m)
		: threshold(_threshold), ballList(_ballList), RayMarchShape(m) {
		lConst = 0;
		for (auto& b : ballList) {
			lConst += b.getRadius();
		}
		lConst = 3.0f / (2.0f*lConst);
	}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

class Displace: public RayMarchShape{
	Shape* A;

	float lConst;
	float alpha;
public:
	Displace(Shape* _A, float _alpha, Material* m) : A(_A), alpha(_alpha), RayMarchShape(m) {
		lConst = 1.0f;
	}
	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};


class InfiniteField : public RayMarchShape {
	Shape* A;

	unsigned int dimentions;
	glm::vec3 repetitionPeriod;
public:
	InfiniteField(Shape* _A, float rP, Material* m) : dimentions(1), A(_A), repetitionPeriod(vec3(rP,0,0)), RayMarchShape(m) {}
	InfiniteField(Shape* _A, vec2 rP, Material* m) : dimentions(2), A(_A), repetitionPeriod(vec3(rP, 0)), RayMarchShape(m) {}
	InfiniteField(Shape* _A, vec3 rP, Material* m) : dimentions(3), A(_A), repetitionPeriod(rP), RayMarchShape(m) {}

	float distance(const glm::vec3& P) const override;
	std::vector<glm::vec3> pointList();
};

#endif