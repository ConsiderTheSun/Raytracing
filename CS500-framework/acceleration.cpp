
#include "geom.h"
//#include "raytrace.h"
#include "acceleration.h"


#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>

/////////////////////////////
// Vector and ray conversions
Ray RayFromBvh(const bvh::Ray<float> &r)
{
    return Ray(vec3FromBvh(r.origin), vec3FromBvh(r.direction));
}
bvh::Ray<float> RayToBvh(const Ray &r)
{
    return bvh::Ray<float>(vec3ToBvh(r.o), vec3ToBvh(r.d));
}


/////////////////////////////
// SimpleBox
bvh::Vector3<float> vec3ToBvh(const vec3& v)
{
    return bvh::Vector3<float>(v[0],v[1],v[2]);
}

vec3 vec3FromBvh(const bvh::Vector3<float>& v)
{
    return vec3(v[0],v[1],v[2]);
}

SimpleBox::SimpleBox(): bvh::BoundingBox<float>() {}
SimpleBox::SimpleBox(const vec3 v): bvh::BoundingBox<float>(vec3ToBvh(v)) {}

SimpleBox& SimpleBox::extend(const vec3 v)
{
    bvh::BoundingBox<float>::extend(vec3ToBvh(v));
    return *this;
}


/////////////////////////////
// BvhShape

SimpleBox BvhShape::bounding_box() const
{
    //  Return the shape's bounding box.
    std::vector<glm::vec3> points = shape->pointList();
    SimpleBox sb = SimpleBox();
    for (vec3& point : points) {
        sb.extend(point);
    }
    return sb;
}

bvh::Vector3<float> BvhShape::center() const
{
    return bounding_box().center();
}
    
std::optional<Intersection> BvhShape::intersect(const bvh::Ray<float>& bvhray) const
{
    // Intersect RayFromBvh(bvhray) with shape;  store result in an Intersection
    // If no intersection,
    //    return std::nullopt;
    // If intersection's t value < bvhray.tmin  or > bvhray.tmax
    //    return std::nullopt;
    // else return
    //    return the Intersection

    Intersection i = shape->intersect(RayFromBvh(bvhray));

    if (i.miss || i.t < bvhray.tmin || i.t > bvhray.tmax)
        return std::nullopt;
    else
        return i;
}

AccelerationBvh::AccelerationBvh(std::vector<Shape*> &objs)
{
    // Wrap all Shape*'s with a bvh specific instance
    for (Shape* shape:  objs) {
        shapeVector.emplace_back(shape); }

    // Magic found in the bvh examples:
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(shapeVector.data(),
                                                                     shapeVector.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), shapeVector.size());

    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), shapeVector.size());
}

Intersection AccelerationBvh::intersect(const Ray& ray)
{
    bvh::Ray<float> bvhRay = RayToBvh(ray);

    // Magic found in the bvh examples:
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, BvhShape> intersector(bvh, shapeVector.data());
    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(bvh);

    auto hit = traverser.traverse(bvhRay, intersector);
    if (hit) {
        return hit->intersection;
    }
    else
        return  Intersection();  // Return an IntersectionRecord which indicates NO-INTERSECTION


}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////


Interval Slab::intersect(Ray r) {
    const float NQ = dot(n, r.o);
    const float ND = dot(n, r.d);
    // Ray intersects both slab planes
    if (abs(ND) > std::numeric_limits<float>::epsilon()) {
        float t0 = -(d0 + NQ) / ND;
        float t1 = -(d1 + NQ) / ND;
        if (t0 > t1) {
            return Interval(t1, t0, -n, n);
        }
        return Interval(t0, t1, -n, n);
    }
    // Ray is parallel to slab planes
    else {
        float s0 = NQ + d0;
        float s1 = NQ + d1;
        if ((s0 < 0 && s1 > 0) || (s0 > 0 && s1 < 0)) {
            return Interval(0, std::numeric_limits<float>::infinity(), -n, n);
        }
        else {
            return Interval(1, 0);
        }
    }
}

Intersection Sphere::intersect(Ray r) {
    glm::vec3 qBar = r.o - center;
    float bHalf = dot(qBar, r.d);
    float c = dot(qBar, qBar) - pow(radius, 2);
    float discriminant = pow(bHalf, 2) - c;

    if (discriminant < 0) return Intersection();

    float sqrtD = sqrt(discriminant);
    float t1 = -bHalf + sqrtD;
    float t2 = -bHalf - sqrtD;

    if (t1 < 0 && t2 < 0) {
        return Intersection();
    }
    else if (t1 < t2) {
        return Intersection(this, t1, normalize((r.o + t1 * r.d) - center));
    }
    else {
        return Intersection(this, t2, normalize((r.o + t2 * r.d) - center));
    }
};

Intersection Box::intersect(Ray r) {
    Slab slabs[] = {
        Slab(glm::vec3(1, 0, 0), -corner.x, -corner.x - diagonal.x),
        Slab(glm::vec3(0, 1, 0), -corner.y, -corner.y - diagonal.y),
        Slab(glm::vec3(0, 0, 1), -corner.z, -corner.z - diagonal.z)
    };
    Interval interval;
    for (Slab slab : slabs) {
        const Interval newInterval = slab.intersect(r);
        if (newInterval.t0 > interval.t0) {
            interval.t0 = newInterval.t0;
            interval.n0 = newInterval.n0;
        }
        if (newInterval.t1 < interval.t1) {
            interval.t1 = newInterval.t1;
            interval.n1 = newInterval.n1;
        }
    }

    if (interval.t0 > interval.t1) {
        return Intersection();
    }
    else if (interval.t0 > 0) {
        return Intersection(this, interval.t0, -interval.n0);
    }
    else if (interval.t1 > 0) {
        return Intersection(this, interval.t1, -interval.n1);
    }
    else {
        return Intersection();
    }
}

Intersection Cylinder::intersect(Ray r) {
    glm::vec3 aBar = normalize(axis);

    glm::vec3 bTemp = cross(glm::vec3(0, 0, 1), aBar);
    if (glm::length(bTemp) < std::numeric_limits<float>::epsilon()) bTemp = cross(glm::vec3(1, 0, 0), aBar);
    glm::vec3 bBar = normalize(bTemp);

    glm::vec3 cBar = cross(aBar, bBar);
    glm::mat3 R = glm::transpose(glm::mat3(bBar, cBar, aBar));

    glm::vec3 qBar = R * (r.o - base);
    glm::vec3 dBar = R * r.d;

    Ray rBar(qBar, dBar);
    Slab slab1(glm::vec3(0, 0, 1), 0, -glm::length(axis));

    float a = pow(dBar.x, 2) + pow(dBar.y, 2);
    float b = 2 * (dBar.x * qBar.x + dBar.y * qBar.y);
    float c = pow(qBar.x, 2) + pow(qBar.y, 2) - pow(radius, 2);
    float discriminant = pow(b, 2) - 4 * a * c;

    if (discriminant < 0) return Intersection();


    Interval i1 = slab1.intersect(rBar);
    Interval i2 = Interval(
        (-b - sqrt(discriminant)) / (2 * a),
        (-b + sqrt(discriminant)) / (2 * a));

    Interval interval;
    if (i1.t0 > i2.t0) {
        interval.t0 = i1.t0;
        interval.n0 = glm::normalize(glm::transpose(R) * glm::vec3(0, 0, -1));
    }
    else {
        interval.t0 = i2.t0;
        interval.n0 = glm::normalize(glm::transpose(R) * glm::vec3(
            qBar.x + i2.t0 * dBar.x,
            qBar.y + i2.t0 * dBar.y, 0));
    }
    if (i1.t1 < i2.t1) {
        interval.t1 = i1.t1;
        interval.n1 = glm::normalize(glm::transpose(R) * glm::vec3(0, 0, 1));
    }
    else {
        interval.t1 = i2.t1;
        interval.n1 = glm::normalize(glm::transpose(R) * glm::vec3(
            qBar.x + i2.t1 * dBar.x,
            qBar.y + i2.t1 * dBar.y, 0));
    }

    if (interval.t0 > interval.t1) {
        return Intersection();
    }
    else if (interval.t0 > 0) {
        return Intersection(this, interval.t0, interval.n0);
    }
    else if (interval.t1 > 0) {
        return Intersection(this, interval.t1, interval.n1);
    }
    else {
        return Intersection();
    }
};

Intersection Tri::intersect(Ray r) {
    Intersection bestIntersection= Intersection(this,std::numeric_limits<float>::infinity(),glm::vec3());
    bestIntersection.miss = true;
    for (ivec3 tri : mesh->triangles) {
        const VertexData& V0 = mesh->vertices[tri[0]];
        const VertexData& V1 = mesh->vertices[tri[1]];
        const VertexData& V2 = mesh->vertices[tri[2]];
        
        const glm::vec3 E1 = V1.pnt - V0.pnt;
        const glm::vec3 E2 = V2.pnt - V0.pnt;
        
        const glm::vec3 p = cross(r.d, E2);
        float d = dot(p, E1);
        
        if (abs(d) < std::numeric_limits<float>::epsilon()) continue;

        const glm::vec3 S = r.o - V0.pnt;
        float u = dot(p, S) / d;
        
        if (u < 0 || u>1) continue;
        
        glm::vec3 q = cross(S, E1);
        float v = dot(r.d, q) / d;
        if (v < 0 || (u + v)>1) continue;
        float t = dot(E2, q) / d;
        if (t <= 0) continue;
      
        glm::vec3 n = (1 - u - v) * V0.nrm + u * V1.nrm + v * V2.nrm;  
        if (t < bestIntersection.t) {
            bestIntersection = Intersection(this, t, n);
        }
    }

    if (bestIntersection.miss) return Intersection();
    return bestIntersection;
};



std::vector<glm::vec3> Sphere::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(center + glm::vec3(radius));
    pointList.push_back(center - glm::vec3(radius));
    return pointList;
}

std::vector<glm::vec3> Box::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(corner);
    pointList.push_back(corner + diagonal);

    return pointList;
}

std::vector<glm::vec3> Cylinder::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(base + glm::vec3(radius));
    pointList.push_back(base - glm::vec3(radius));

    pointList.push_back(axis+base + glm::vec3(radius));
    pointList.push_back(axis+base - glm::vec3(radius));


    return pointList;
}

std::vector<glm::vec3> Tri::pointList() {
    std::vector<glm::vec3> pointList;
    for (VertexData& v : mesh->vertices) {
        pointList.push_back(v.pnt);
    }

    return pointList;
}
