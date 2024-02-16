
#include "geom.h"
#include "acceleration.h"

#include "AuxilaryFunctions.h"
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>

#include "raytrace.h"


constexpr float EPSILON = 0.001;
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


//#define GGX


glm::vec3 Shape::SampleBrdf(glm::vec3 omegaO, glm::vec3 N){

    const float xi = AuxilaryFunctions::random();
    const float xi1 = AuxilaryFunctions::random();
    const float xi2 = AuxilaryFunctions::random();

    float pd = glm::length(material->Kd);
    float pr = glm::length(material->Ks);
    float pt = glm::length(material->Kt);

    const float s = pd + pr + pt;

    pd /= s;
    pr /= s;
    pt /= s;

    // diffuse 
    if (xi < pd) {
        return AuxilaryFunctions::SampleLobe(N, sqrt(xi1), 2 * M_PI * xi2);
    }

    // calculates m, same for reflection and transmission
#ifdef GGX
    const float cosThetaM = cos(atan2(material->alpha * sqrt(xi1), sqrt(1 - xi1)));
#else
    const float cosThetaM = pow(xi1, 1.0f / (material->alpha + 1));
#endif

    const glm::vec3 m = AuxilaryFunctions::SampleLobe(N, cosThetaM, 2 * M_PI * xi2);

    // reflection
    if(xi < pd + pr) {
        return 2.0f * abs(dot(omegaO, m)) * m - omegaO;
    }
    // transmission
    else {
        // calculates the etas
        float eta, etaI, etaO;
        if (dot(omegaO, N) > 0) {
            etaI = 1.0f;
            etaO = material->ior;
        }
        else {
            etaI = material->ior;
            etaO = 1.0f;
        }
        eta = etaI / etaO;

        float a = dot(omegaO, m);
        float powy = pow(dot(omegaO, m),2);
        float b = (1.0f - pow(dot(omegaO, m), 2));
        float c = pow(eta, 2) * (1.0f - pow(dot(omegaO, m), 2));


        // checks for total internal reflection
        const float radicand = 1.0 - pow(eta, 2) * (1.0f - pow(dot(omegaO, m), 2));
        // negative => tir
        if (radicand < 0) {
            return 2.0f * abs(dot(omegaO, m)) * m - omegaO;
        }
        else {
            return (eta*dot(omegaO,m) - AuxilaryFunctions::sign(dot(omegaO,N))*sqrt(radicand)) 
                * m - eta*omegaO;
        }
    }
}

float Shape::PdfBrdf(glm::vec3 omegaO, glm::vec3 N, glm::vec3 omegaI) {
    // calculates the etas
    float eta, etaI, etaO;
    if (dot(omegaO, N) > 0) {
        etaI = 1.0f;
        etaO = material->ior;
    }
    else {
        etaI = material->ior;
        etaO = 1.0f;
    }
    eta = etaI / etaO;

    // calculates the m values
    const glm::vec3 mr = normalize(omegaO + omegaI);
    const glm::vec3 mt = -normalize(etaO * omegaI + etaI * omegaO);

    // calculates the weights
    float pd = glm::length(material->Kd);
    float pr = glm::length(material->Ks);
    float pt = glm::length(material->Kt);

    const float s = pd + pr + pt;

    pd /= s;
    pr /= s;
    pt /= s;

    const float Pd = abs(dot(omegaI, N)) / M_PI;
    const float Pr = D(mr,N) * abs(dot(mr, N)) / (4 * abs(dot(omegaI, mr)));

    // checks for total internal reflection
    const float radicand = 1.0 - pow(eta, 2) * (1.0f - pow(dot(omegaO, mt), 2));
    // negative => tir
    float Pt;
    if (radicand < 0) {
        Pt = Pr;
    }
    else {
        Pt = D(mt, N) * abs(dot(mt, N)) * ((pow(etaO,2) * abs(dot(omegaI,mt)))
            / pow( etaO*dot(omegaI, mt) + etaI*dot(omegaO,mt),2));
    }

    return pd*Pd + pr*Pr + pt*Pt;
}

glm::vec3 Shape::EvalScattering(glm::vec3 omegaO, glm::vec3 N, glm::vec3 omegaI, float attenuationDistance) {
    // calculates the etas
    float eta, etaI, etaO;
    if (dot(omegaO, N) > 0) {
        etaI = 1.0f;
        etaO = material->ior;
    }
    else {
        etaI = material->ior;
        etaO = 1.0f;
    }
    eta = etaI / etaO;

    // calculates the m values
    const glm::vec3 mr = normalize(omegaO + omegaI);
    const glm::vec3 mt = -normalize(etaO * omegaI + etaI * omegaO);

    // calculates the weights
    float pd = glm::length(material->Kd);
    float pr = glm::length(material->Ks);
    float pt = glm::length(material->Kt);

    const float s = pd + pr + pt;

    pd /= s;
    pr /= s;
    pt /= s;

    // default value if prob is too low
    vec3 Ed = vec3(0);
    vec3 Er = vec3(0);
    vec3 Et = vec3(0);

    if (pd > 0.001) {
        Ed = material->Kd / M_PI;
    }
    if (pr > 0.001) {
        Er = (D(mr, N) * G(omegaI, omegaO, mr, N) * F(glm::normalize(dot(omegaI, mr))))
            / (4 * abs(dot(omegaI, N)) * abs(dot(omegaO, N)));
    }
    if (pt > 0.001) {
        vec3 attenuationFactor;

        if (dot(omegaO, N) < 0) {
            const vec3 logKt = vec3(std::log(material->Kt.x), std::log(material->Kt.y), std::log(material->Kt.z))
                / std::log(exp(1.0));
            attenuationFactor = vec3(
                pow(std::exp(1.0), attenuationDistance * logKt.x),
                pow(std::exp(1.0), attenuationDistance * logKt.y),
                pow(std::exp(1.0), attenuationDistance * logKt.z));
        }
        else {
            attenuationFactor = vec3(1.0f);
        }



        // checks for total internal reflection
        const float radicand = 1.0 - pow(eta, 2) * (1.0f - pow(dot(omegaO, mt), 2));
        // negative => tir
        if (radicand < 0) {

            Et = attenuationFactor* (D(mr, N) * G(omegaI, omegaO, mr, N) * F(glm::normalize(dot(omegaI, mr))))
                / (4 * abs(dot(omegaI, N)) * abs(dot(omegaO, N)));
        }
        else {
            Et = attenuationFactor * (D(mt, N) * G(omegaI, omegaO, mt, N) * (vec3(1)-F(glm::normalize(dot(omegaI, mt)))))
                / (abs(dot(omegaI, N)) * abs(dot(omegaO, N)));
            Et *= abs(dot(omegaI, mt)) * abs(dot(omegaO, mt)) * pow(etaO, 2) /
                pow(etaO * dot(omegaI, mt) + etaI * dot(omegaO, mt), 2);
        }
    }
    

    return abs(dot(N, omegaI)) * (Ed + Er + Et);
}

float Shape::Kai(float d) {
    if (d > 0) return 1;
    else       return 0;
}

float Shape::D(glm::vec3 m, glm::vec3 N){
#ifdef GGX
        const float ag2 = pow(material->alpha, 2);
        const float tan2ThetaM = pow(sqrt(1.0f - pow(dot(m, N), 2)) / dot(m, N), 2);
        const float ggxD = Kai(dot(m, N)) * ag2 /
            (M_PI * pow(dot(N, m),4) * pow(ag2+ tan2ThetaM,2));
        return ggxD;
#else
    const float phongD = Kai(dot(m, N)) * ((material->alpha + 2) / (2 * M_PI)) * pow(dot(m, N), material->alpha);
    return phongD;
#endif
}

float Shape::G(glm::vec3 omegaI, glm::vec3 omegaO, glm::vec3 m, glm::vec3 N){
    return G1(omegaI, m, N) * G1(omegaO, m, N);
}
float Shape::G1(glm::vec3 v, glm::vec3 m, glm::vec3 N){

    // round off error checks
    const float vN = dot(v, N);
    if (vN > 1.0f) return 1.0f;

    const float tanThetaV = sqrt(1.0 - pow(vN, 2)) / vN;
    if (tanThetaV < EPSILON) return 1.0f;

    const float kaiTerm = Kai(dot(v, m) / vN);

#ifdef GGX
    const float ag2 = pow(material->alpha, 2);
    return kaiTerm * 2 / (1 + sqrt(1 + ag2 * pow(tanThetaV, 2)));
#else
    const float a = sqrt(material->alpha / 2 + 1) / tanThetaV;
    if (a > 1.6) return kaiTerm;

    return kaiTerm * (3.535 * a + 2.181 * pow(a, 2)) / (1.0 + 2.276 * a + 2.577 * pow(a, 2));
#endif
}

glm::vec3 Shape::F(float d) {
    return material->Ks + ((1.0f - material->Ks) * (float)pow(1 - abs(d), 5));
}

Color Shape::EvalRadiance() {
    return material->Kd; 
}




Interval Slab::intersect(Ray r) {
    const float NQ = dot(n, r.o);
    const float ND = dot(n, r.d);
    // Ray intersects both slab planes
    if (abs(ND) > EPSILON) {
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

    if ((t1 < t2 || t2 < EPSILON) && t1 > EPSILON) {
        return Intersection(this, t1, normalize((r.o + t1 * r.d) - center), r.o + r.d*t1);
    }
    else if (t2 > EPSILON) {
        return Intersection(this, t2, normalize((r.o + t2 * r.d) - center), r.o + r.d * t2);
    }
    else {
        return Intersection();
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
    else if (interval.t0 > EPSILON) {
        return Intersection(this, interval.t0, -interval.n0, r.o + r.d * interval.t0);
    }
    else if (interval.t1 > EPSILON) {
        return Intersection(this, interval.t1, -interval.n1, r.o + r.d * interval.t1);
    }
    else {
        return Intersection();
    }
}

Intersection Cylinder::intersect(Ray r) {
    glm::vec3 aBar = normalize(axis);

    glm::vec3 bTemp = cross(glm::vec3(0, 0, 1), aBar);
    if (glm::length(bTemp) < EPSILON) bTemp = cross(glm::vec3(1, 0, 0), aBar);
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
    else if (interval.t0 > EPSILON) {
        return Intersection(this, interval.t0, interval.n0, r.o + r.d * interval.t0);
    }
    else if (interval.t1 > EPSILON) {
        return Intersection(this, interval.t1, interval.n1, r.o + r.d * interval.t1);
    }
    else {
        return Intersection();
    }
};

Intersection Tri::intersect(Ray r) {
    Intersection bestIntersection= Intersection(this,std::numeric_limits<float>::infinity(),glm::vec3(),glm::vec3());
    bestIntersection.miss = true;
    for (ivec3 tri : mesh->triangles) {
        const VertexData& V0 = mesh->vertices[tri[0]];
        const VertexData& V1 = mesh->vertices[tri[1]];
        const VertexData& V2 = mesh->vertices[tri[2]];
        
        const glm::vec3 E1 = V1.pnt - V0.pnt;
        const glm::vec3 E2 = V2.pnt - V0.pnt;
        
        const glm::vec3 p = cross(r.d, E2);
        float d = dot(p, E1);
        
        if (abs(d) < EPSILON) continue;

        const glm::vec3 S = r.o - V0.pnt;
        float u = dot(p, S) / d;
        
        if (u < 0 || u>1) continue;
        
        glm::vec3 q = cross(S, E1);
        float v = dot(r.d, q) / d;
        if (v < 0 || (u + v)>1) continue;
        float t = dot(E2, q) / d;
        if (t <= EPSILON) continue;// TODO: make sure this is right
      
        glm::vec3 n = (1 - u - v) * V0.nrm + u * V1.nrm + v * V2.nrm;  
        if (t < bestIntersection.t) {
            bestIntersection = Intersection(this, t, n, r.o + r.d * t);
        }
    }
    
    if (bestIntersection.miss) return Intersection();
    return bestIntersection;
};


Intersection RayMarchShape::intersect(Ray r) {
    

    float t = 0.01;
    for (int stepCount = 0; true; ++stepCount) {
        const glm::vec3 P = r.o + r.d * t;
        const float dt = this->distance(P);

        t += abs(dt);

        // termination conditions

        if (abs(dt) < 0.00001) break;
        if (stepCount > 2500) return Intersection();
        if(t >= 10000) return Intersection();
    }

    if(t<0.01) return Intersection();

    // normal calculation
    const glm::vec3 P = r.o + r.d * t;
    const float h = 0.01;
    const float nx = distance(P + glm::vec3(h, 0, 0)) - distance(P - glm::vec3(h, 0, 0));
    const float ny = distance(P + glm::vec3(0, h, 0)) - distance(P - glm::vec3(0, h, 0));
    const float nz = distance(P + glm::vec3(0, 0, h)) - distance(P - glm::vec3(0, 0, h));

    const glm::vec3 N = glm::normalize(glm::vec3(nx, ny, nz));

    return Intersection(this, t, N, P);
}




float Sphere::distance(const glm::vec3& P) const{
    return glm::length(P - center) - radius;
}

float Box::distance(const glm::vec3& P) const {
    glm::vec3 max = corner + diagonal;
    return std::max(P.x - max.x, 
        std::max(corner.x - P.x, 
            std::max(P.y - max.y, 
                std::max(corner.y - P.y, 
                    std::max(P.z - max.z, corner.z - P.z)))));
}

float Cylinder::distance(const glm::vec3& P) const {
    vec2 d = glm::abs(vec2(length(glm::vec2(P.x, P.y)), P.z- axis.z/2)) - vec2(radius, axis.z/2);
    return std::min(std::max(d.x, d.y), 0.0f) +length(glm::max(d, vec2(0.0)));


    float a = glm::length(glm::vec2(P.x, P.y)) - radius;
    float b = 0;
    if (P.z > axis.z) {
        b = P.z - axis.z;
    }
    else if (P.z < 0) {
        b = -P.z;
    }
    return sqrt(pow(a,2) + pow(b,2));
}

float Torus::distance(const glm::vec3& P) const {
    glm::vec3 p = P - center;
    float m = sqrt(p.x * p.x + p.y * p.y) - R;
    return sqrt(m * m + p.z * p.z) - r;
}

float Cone::distance(const glm::vec3& P) const {

    vec2 d = glm::abs(vec2(length(glm::vec2(P.x, P.y)), P.z - height / 2)) - vec2(radius, height / 2);
    float cylinderCutoff = std::min(std::max(d.x, d.y), 0.0f) + length(glm::max(d, vec2(0.0)));
    
    glm::vec3 p = P - center;
    float infCone = glm::length(glm::vec2(p.x, p.y)) * cos(theta) - abs(p.z) * sin(theta);

    return std::max(infCone, cylinderCutoff);
    return infCone;
}

float Pyramid::distance(const glm::vec3& P) const {
    float m2 = height * height + 0.25;

    vec3 p = P;

    p.xz = glm::abs(vec2(p.xz));
    p.xz = (p.z > p.x) ? vec2(p.zx) : vec2(p.xz);
    p.xz -= vec2(0.5f);

    vec3 q = vec3(p.z, height * p.y - 0.5 * p.x, height * p.x + 0.5 * p.y);

    float s = std::max(-q.x, 0.0f);
    float t = std::clamp((q.y - 0.5 * p.z) / (m2 + 0.25), 0.0, 1.0);

    float a = m2 * (q.x + s) * (q.x + s) + q.y * q.y;
    float b = m2 * (q.x + 0.5 * t) * (q.x + 0.5 * t) + (q.y - m2 * t) * (q.y - m2 * t);

    float d2 = std::min(q.y, -q.x * m2 - q.y * 0.5f) > 0.0 ? 0.0 : std::min(a, b);

    return sqrt((d2 + q.z * q.z) / m2) * AuxilaryFunctions::sign(std::max(q.z, -p.y));
}

float Octohedron::distance(const glm::vec3& P) const {
    vec3 pAbs = glm::abs(P);
    return (pAbs.x + pAbs.y + pAbs.z - s) * 0.57735027;
}

float TriPrism::distance(const glm::vec3& P) const {
    vec3 pAbs = glm::abs(P);
    return std::max(pAbs.z - height, std::max(pAbs.x * 0.866025f + P.y * 0.5f, -P.y) - side * 0.5f);
}

float HexPrism::distance(const glm::vec3& P) const {
    vec3 pAbs = glm::abs(P);

    const vec3 k = vec3(-0.8660254, 0.5, 0.57735);
    pAbs.xy -= 2.0f * glm::min(glm::dot(vec2(k.x,k.y), vec2(pAbs.x,pAbs.y)), 0.0f) * k.xy;

    float clamp = pAbs.x;
    if (clamp > k.z * side) clamp = k.z * side;
    else if (clamp < -k.z * side) clamp = -k.z * side;

    vec2 d = vec2(
        length(pAbs.xy - vec2(clamp, side)) * AuxilaryFunctions::sign(pAbs.y - side),
        pAbs.z - height);
    return std::min(std::max(d.x, d.y), 0.0f) + length(glm::max(d, 0.0f));
}

float Link::distance(const glm::vec3& P) const {
    vec3 q = vec3(P.x, std::max(abs(P.y) - le, 0.0f), P.z);
    return length(vec2(length(vec2(q.xy)) - r1, q.z)) - r2;
}


float Translate::distance(const glm::vec3& P) const {
    return A->distance(P - position);
}

float Rotate::distance(const glm::vec3& P) const {
    return A->distance((Rinv*glm::vec4(P,1)).xyz);
}

float Scale::distance(const glm::vec3& P) const {
    return lConst * A->distance(glm::vec3(P)/scale);
}

float Twist::distance(const glm::vec3& P) const {
    glm::vec3 pTwist;
    pTwist.x = P.x * cos(twistSpeed * P.z) - P.y * sin(twistSpeed * P.z);
    pTwist.y = P.x * sin(twistSpeed * P.z) + P.y * cos(twistSpeed * P.z);
    pTwist.z = P.z;
    return lConst * A->distance(pTwist);
}

float MetaBalls::distance(const glm::vec3& P) const {

    //return ballList[0].distance(P);

    float finalDistance = threshold;
    

    for (auto& b : ballList) {
        float d = glm::length(P - b.getCenter());
        float C = 0;
        if (d < b.getRadius()) {
            C = 2 * pow(d,3)/pow(b.getRadius(), 3) -
                3 * pow(d,2)/pow(b.getRadius(), 2) + 1;
        }
        
        finalDistance -= C;
    }
    return lConst*finalDistance;
}



float Displace::distance(const glm::vec3& P) const {
    float d1 = A->distance(P);
    float d2 = sin(alpha * P.x) * sin(alpha * P.y) * sin(alpha * P.z);
    return d1 + d2;
}


float InfiniteField::distance(const glm::vec3& P) const {
    //x/c-floor(x/c)

    //return A->distance(P);

    glm::vec3 repP = P;
    switch (dimentions) {
    case 3:
        repP.z = repetitionPeriod.z * (P.z / repetitionPeriod.z - floor(P.z / repetitionPeriod.z));
    case 2:
        repP.y = repetitionPeriod.y * (P.y / repetitionPeriod.y - floor(P.y / repetitionPeriod.y));
    case 1:
        repP.x = repetitionPeriod.x * (P.x / repetitionPeriod.x - floor(P.x / repetitionPeriod.x));
    default:
        break;
    }
    //glm::vec3 repP = P + 0.5f * repetitionPeriod;
    

    //repP.x = std::fmod(repP.x, repetitionPeriod.x);
    
    //repP.y = std::fmod(repP.y, repetitionPeriod.y);
    //repP.z = std::fmod(repP.z, repetitionPeriod.z);

    //repP -= 0.5f * repetitionPeriod;

    return A->distance(repP);
}


std::vector<glm::vec3> Sphere::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(center + glm::vec3(radius));
    pointList.push_back(center + vec3(radius, radius, -radius));
    pointList.push_back(center + vec3(radius, -radius, radius));
    pointList.push_back(center + vec3(radius, -radius, -radius));
    pointList.push_back(center + vec3(-radius, radius, radius));
    pointList.push_back(center + vec3(-radius, radius, -radius));
    pointList.push_back(center + vec3(-radius, -radius, radius));
    pointList.push_back(center - glm::vec3(radius));

    return pointList;
}

std::vector<glm::vec3> Box::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(corner);

    pointList.push_back(corner + diagonal);

    pointList.push_back(corner + vec3(diagonal.x, -diagonal.y, diagonal.z));
    pointList.push_back(corner + vec3(diagonal.x, -diagonal.y, -diagonal.z));
    pointList.push_back(corner + vec3(diagonal.x, diagonal.y, -diagonal.z));

    pointList.push_back(corner + vec3(-diagonal.x, diagonal.y, diagonal.z));
    pointList.push_back(corner + vec3(-diagonal.x, -diagonal.y, diagonal.z));
    pointList.push_back(corner + vec3(-diagonal.x, diagonal.y, -diagonal.z));

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

std::vector<glm::vec3> Torus::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(center + glm::vec3(R,R,R) + glm::vec3(r,r,r));
    pointList.push_back(center - glm::vec3(R,R,R) - glm::vec3(r,r,r));

    pointList.push_back(center + glm::vec3(0, r, 0));
    pointList.push_back(center - glm::vec3(0, r, 0));

    return pointList;
}

std::vector<glm::vec3> Cone::pointList() {
    std::vector<glm::vec3> pointList;
    pointList.push_back(center);

    pointList.push_back(center + glm::vec3(radius));
    pointList.push_back(center - glm::vec3(radius));
    pointList.push_back(center - height + glm::vec3(radius));
    pointList.push_back(center - height - glm::vec3(radius));

    pointList.push_back(vec3(0,0,height) + center + glm::vec3(radius));
    pointList.push_back(vec3(0, 0, height) + center - glm::vec3(radius));

    return pointList;
}

std::vector<glm::vec3> Pyramid::pointList() {
    std::vector<glm::vec3> pointList;

    pointList.push_back(vec3(height));
    pointList.push_back(vec3(-height));
    pointList.push_back(vec3(2));
    pointList.push_back(vec3(-2));

    return pointList;
}

std::vector<glm::vec3> Octohedron::pointList() {
    std::vector<glm::vec3> pointList;
  
    pointList.push_back(vec3(s));
    pointList.push_back(vec3(-s));

    return pointList;
}

std::vector<glm::vec3> TriPrism::pointList() {
    std::vector<glm::vec3> pointList;

    pointList.push_back(vec3(height));
    pointList.push_back(vec3(-height));
    pointList.push_back(vec3(2 * side));
    pointList.push_back(vec3(-2 * side));

    return pointList;
}

std::vector<glm::vec3> HexPrism::pointList() {
    std::vector<glm::vec3> pointList;

    pointList.push_back(vec3(height));
    pointList.push_back(vec3(-height));
    pointList.push_back(vec3(2 * side));
    pointList.push_back(vec3(-2 * side));

    return pointList;
}

std::vector<glm::vec3> Link::pointList() {
    std::vector<glm::vec3> pointList;

    pointList.push_back(vec3(1000));
    pointList.push_back(vec3(-1000));

    return pointList;
}

std::vector<glm::vec3> Translate::pointList() {
    std::vector<glm::vec3> pointList = A->pointList();
    for (glm::vec3& point : pointList) {
        point += position;
    }
    return pointList;
}

std::vector<glm::vec3> Rotate::pointList() {
    std::vector<glm::vec3> pointList = A->pointList();
    for (glm::vec3& point : pointList) {
        point = (R * glm::vec4(point,1)).xyz;
    }
    return pointList;
}

std::vector<glm::vec3> Scale::pointList() {
    std::vector<glm::vec3> pointList = A->pointList();
    for (glm::vec3& point : pointList) {
        point *= scale;
    }
    return pointList;
}

std::vector<glm::vec3> Twist::pointList() {

    //return A->pointList();



    std::vector<glm::vec3> shapePointList = A->pointList();

    std::vector<glm::vec3> pointList = shapePointList;

    for (const auto& p : shapePointList) {
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), (float)M_PI/2.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), (float)M_PI, vec3(0, 0, 1)) * vec4(p, 1))
        );
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), 3.0f*(float)M_PI/2.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );



        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), (float)M_PI/4.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), 3.0f* (float)M_PI/4.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), 5.0f * (float)M_PI / 4.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );
        pointList.push_back(
            vec3(rotate(glm::mat4(1.0f), 7.0f * (float)M_PI / 4.0f, vec3(0, 0, 1)) * vec4(p, 1))
        );
    }
    return pointList;

    pointList.push_back(glm::vec3(100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, -100000, -100000));

    pointList.push_back(glm::vec3(100000, 100000, -100000));
    pointList.push_back(glm::vec3(100000, -100000, 100000));
    pointList.push_back(glm::vec3(100000, -100000, -100000));
    pointList.push_back(glm::vec3(-100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, 100000, -100000));
    pointList.push_back(glm::vec3(-100000, -100000, 100000));

    return pointList;
}

std::vector<glm::vec3> MetaBalls::pointList() {

    std::vector<glm::vec3> pointList;
    for (Sphere& ball : ballList) {
        for(auto p : ball.pointList())
        pointList.push_back(p);
    }
    return pointList;
}

std::vector<glm::vec3> Displace::pointList() {

    //return A->pointList();

    std::vector<glm::vec3> pointList;
    pointList.push_back(glm::vec3(100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, -100000, -100000));

    pointList.push_back(glm::vec3(100000, 100000, -100000));
    pointList.push_back(glm::vec3(100000, -100000, 100000));
    pointList.push_back(glm::vec3(100000, -100000, -100000));
    pointList.push_back(glm::vec3(-100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, 100000, -100000));
    pointList.push_back(glm::vec3(-100000, -100000, 100000));

    return pointList;
}

std::vector<glm::vec3> InfiniteField::pointList() {

    //return A->pointList();

    std::vector<glm::vec3> pointList;
    pointList.push_back(glm::vec3(100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, -100000, -100000));

    pointList.push_back(glm::vec3(100000, 100000, -100000));
    pointList.push_back(glm::vec3(100000, -100000, 100000));
    pointList.push_back(glm::vec3(100000, -100000, -100000));
    pointList.push_back(glm::vec3(-100000, 100000, 100000));
    pointList.push_back(glm::vec3(-100000, 100000, -100000));
    pointList.push_back(glm::vec3(-100000, -100000, 100000));

    return pointList;
}


Intersection Sphere::SampleSphere() {
    float xi1 = AuxilaryFunctions::random();
    float xi2 = AuxilaryFunctions::random();

    float z = 2 * xi1 - 1;
    float r = sqrt(1 - pow(z, 2));
    float a = 2 * M_PI * xi2;

    glm::vec3 N = glm::vec3(r * cos(a), r * sin(a), z);
    glm::vec3 P = center + radius * N;
    return Intersection(this, 0, N, P);
}