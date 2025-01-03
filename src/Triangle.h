#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Vec3.h"
#include "Ray.h"
#include "Plane.h"

struct RayTriangleIntersection{
    bool intersectionExists;
    float t;
    float w0,w1,w2;
    unsigned int tIndex;
    Vec3 intersection;
    Vec3 normal;
};

class Triangle {
private:
    Vec3 m_c[3] , m_normal;
    float area;
public:
    Triangle() {}
    Triangle( Vec3 const & c0 , Vec3 const & c1 , Vec3 const & c2 ) {
        m_c[0] = c0;
        m_c[1] = c1;
        m_c[2] = c2;
        updateAreaAndNormal();
    }

    Vec3 getM_c(int indice){
        return m_c[indice];
    }

    Vec3 centroid() const {
        return (m_c[0] + m_c[1] + m_c[2]) / 3.0f;
    }


    void updateAreaAndNormal() {
        Vec3 nNotNormalized = Vec3::cross( m_c[1] - m_c[0] , m_c[2] - m_c[0] );
        float norm = nNotNormalized.length();
        m_normal = nNotNormalized / norm;
        area = norm / 2.f;
    }
    void setC0( Vec3 const & c0 ) { m_c[0] = c0; }
    void setC1( Vec3 const & c1 ) { m_c[1] = c1; }
    void setC2( Vec3 const & c2 ) { m_c[2] = c2; }
    Vec3 const & normal() const { return m_normal; }

    Vec3 projectOnSupportPlane( Vec3 const & p ) const {
        float d = Vec3::dot(m_normal, m_c[0]); 
        float t = (d - Vec3::dot(m_normal, p)) / Vec3::dot(m_normal, m_normal);
        return p + t * m_normal;
    }

    float squareDistanceToSupportPlane( Vec3 const & p ) const {
        float d = Vec3::dot(m_normal, m_c[0]); 
        float dist = Vec3::dot(m_normal, p) - d;
        return dist * dist;
    }

    bool isParallelTo( Line const & L ) const {
        return std::abs(Vec3::dot(L.direction(), m_normal)) < 1e-6f;
    }

    Vec3 getIntersectionPointWithSupportPlane( Line const & L ) const {

        float d = Vec3::dot(m_normal, m_c[0]);
        float t = (d - Vec3::dot(m_normal, L.origin())) / Vec3::dot(m_normal, L.direction());
        return L.origin() + t * L.direction();
    }

    void computeBarycentricCoordinates( Vec3 const & p , float & u0 , float & u1 , float & u2 ) const {
        Vec3 v0 = m_c[1] - m_c[0];
        Vec3 v1 = m_c[2] - m_c[0];
        Vec3 v2 = p - m_c[0];

        float d00 = Vec3::dot(v0, v0);
        float d01 = Vec3::dot(v0, v1);
        float d11 = Vec3::dot(v1, v1);
        float d20 = Vec3::dot(v2, v0);
        float d21 = Vec3::dot(v2, v1);

        float denom = d00 * d11 - d01 * d01;
        u1 = (d11 * d20 - d01 * d21) / denom;
        u2 = (d00 * d21 - d01 * d20) / denom;
        u0 = 1.0f - u1 - u2;
    }

    RayTriangleIntersection getIntersection( Ray const & ray ) const {
        RayTriangleIntersection result;
        result.intersectionExists = false;

        if (isParallelTo(ray)) {
            return result;
        }

        Vec3 intersectionPoint = getIntersectionPointWithSupportPlane(ray);
        
        Vec3 direction = intersectionPoint - ray.origin();
        float t = direction.length();
        
        if (Vec3::dot(direction, ray.direction()) < 0) {
            return result;
        }

        float w0, w1, w2;
        computeBarycentricCoordinates(intersectionPoint, w0, w1, w2);

        if (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f && 
            std::abs(w0 + w1 + w2 - 1.0f) < 1e-5f) {
            result.intersectionExists = true;
            result.intersection = intersectionPoint;
            result.t = t;
            result.w0 = w0;
            result.w1 = w1;
            result.w2 = w2;
            result.normal = m_normal;
        }

        return result;
    }

};
#endif