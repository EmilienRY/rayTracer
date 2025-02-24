#ifndef Sphere_H
#define Sphere_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySphereIntersection{
    bool intersectionExists;
    float t;
    float theta,phi;
    Vec3 intersection;
    Vec3 secondintersection;
    Vec3 normal;
};


static
Vec3 SphericalCoordinatesToEuclidean( float theta , float phi ) {
    return Vec3( cos(theta) * cos(phi) , sin(theta) * cos(phi) , sin(phi) );
}


float dot(Vec3 a,Vec3 b){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

 
class Sphere : public Mesh {
public:
    Vec3 m_center;
    float m_radius;

    Sphere() : Mesh() {}
    Sphere(Vec3 c , float r) : Mesh() , m_center(c) , m_radius(r) {}

    void build_arrays(){
        unsigned int nTheta = 20 , nPhi = 20;
        positions_array.resize(3 * nTheta * nPhi );
        normalsArray.resize(3 * nTheta * nPhi );
        uvs_array.resize(2 * nTheta * nPhi );
        for( unsigned int thetaIt = 0 ; thetaIt < nTheta ; ++thetaIt ) {
            float u = (float)(thetaIt) / (float)(nTheta-1);
            float theta = u * 2 * M_PI;
            for( unsigned int phiIt = 0 ; phiIt < nPhi ; ++phiIt ) {
                unsigned int vertexIndex = thetaIt + phiIt * nTheta;
                float v = (float)(phiIt) / (float)(nPhi-1);
                float phi = - M_PI/2.0 + v * M_PI;
                Vec3 xyz = SphericalCoordinatesToEuclidean( theta , phi );
                positions_array[ 3 * vertexIndex + 0 ] = m_center[0] + m_radius * xyz[0];
                positions_array[ 3 * vertexIndex + 1 ] = m_center[1] + m_radius * xyz[1];
                positions_array[ 3 * vertexIndex + 2 ] = m_center[2] + m_radius * xyz[2];
                normalsArray[ 3 * vertexIndex + 0 ] = xyz[0];
                normalsArray[ 3 * vertexIndex + 1 ] = xyz[1];
                normalsArray[ 3 * vertexIndex + 2 ] = xyz[2];
                uvs_array[ 2 * vertexIndex + 0 ] = u;
                uvs_array[ 2 * vertexIndex + 1 ] = v;
            }
        }
        triangles_array.clear();
        for( unsigned int thetaIt = 0 ; thetaIt < nTheta - 1 ; ++thetaIt ) {
            for( unsigned int phiIt = 0 ; phiIt < nPhi - 1 ; ++phiIt ) {
                unsigned int vertexuv = thetaIt + phiIt * nTheta;
                unsigned int vertexUv = thetaIt + 1 + phiIt * nTheta;
                unsigned int vertexuV = thetaIt + (phiIt+1) * nTheta;
                unsigned int vertexUV = thetaIt + 1 + (phiIt+1) * nTheta;
                triangles_array.push_back( vertexuv );
                triangles_array.push_back( vertexUv );
                triangles_array.push_back( vertexUV );
                triangles_array.push_back( vertexuv );
                triangles_array.push_back( vertexUV );
                triangles_array.push_back( vertexuV );
            }
        }
    }



    RaySphereIntersection intersect(const Ray &ray) const {
        RaySphereIntersection intersection;

        Vec3 origin =ray.origin();
        Vec3 direction =ray.direction();


        float a=dot(direction,direction);

        Vec3 oc=origin-m_center;
        float b=2.0*dot(direction,oc);

        float c=dot(oc,oc)-m_radius*m_radius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) { 
            intersection.intersectionExists=false;
            return intersection;
        } else {
            float t1 = (-b - std::sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + std::sqrt(discriminant)) / (2.0f * a);
            Vec3 point_intersection1= origin+t1*direction;
            Vec3 point_intersection2= origin+t2*direction;

            if(t1<t2){
                if(t1>1e-4){
                    intersection.intersectionExists=true;
                    intersection.t=t1;
                    intersection.intersection=point_intersection1;
                    intersection.secondintersection=point_intersection2;

                }
                else{
                    if(t2>1e-4){
                        intersection.intersectionExists=true;
                        intersection.t=t2;
                        intersection.intersection=point_intersection2;
                        intersection.secondintersection=point_intersection1;
                    }
                    else{
                        intersection.intersectionExists=false;

                    }

                }
            }else{
                if(t2>1e-4){
                    intersection.intersectionExists=true;
                    intersection.t=t2;
                    intersection.intersection=point_intersection2;
                    intersection.secondintersection=point_intersection1;
                }
                else{
                    if(t1>1e-4){
                        intersection.intersectionExists=true;
                        intersection.t=t1;
                        intersection.intersection=point_intersection1;
                        intersection.secondintersection=point_intersection2;
                    }
                    else{
                        intersection.intersectionExists=false;

                    }

                }
            }

            if(intersection.intersectionExists){
                Vec3 norm=intersection.intersection - m_center;
                norm.normalize();
                intersection.normal= norm;
            } 


            return intersection;
        }

        return intersection;
    }


    // RaySphereIntersection intersect(const Ray &ray) const {
    //     RaySphereIntersection intersection;
    //     Vec3 origine=ray.origin();
    //     Vec3 direction=ray.direction();
    //     float a=dot(direction,direction);
    //     float b=2*dot(direction,(origine-this->m_center));
    //     float c=dot(origine-this->m_center,origine-this->m_center)-m_radius*m_radius;
    //     float discriminent=b*b-4*a*c;
    //     if(discriminent>0){
    //         intersection.intersectionExists=true;
    //         float t1=(-b-sqrt(discriminent))/(2*a);
    //         float t2=(-b+sqrt(discriminent))/(2*a);
    //         if(t1>1e-4 && t2>1e-4){
    //             intersection.t=std::min(t1, t2);
    //             intersection.intersection=origine + intersection.t *direction;
    //             intersection.normal=intersection.intersection - m_center;
    //             intersection.normal.normalize();
    //         }
    //         else if(t1>1e-4 && t2<1e-4){
    //             intersection.t=t1;
    //             intersection.intersection=origine + t1 *direction;
    //             intersection.normal=intersection.intersection - m_center;
    //             intersection.normal.normalize();
    //         }
    //         else if (t2>1e-4 && t1<1e-4){
    //             intersection.t=t2;
    //             intersection.intersection=origine + t2 *direction;
    //             intersection.normal=intersection.intersection - m_center;
    //             intersection.normal.normalize();
    //         }
    //         else{
    //             intersection.intersectionExists=false;
    //         }
            
    //     }
    //    else{
    //         intersection.intersectionExists=false;
    //     }
    //     return intersection;
    // }


};
#endif
