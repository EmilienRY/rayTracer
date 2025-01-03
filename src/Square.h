#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySquareIntersection{
    bool intersectionExists;
    float t;
    float u,v;
    Vec3 intersection;
    Vec3 normal;
};


class Square : public Mesh {
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
           float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) : Mesh() {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad( Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
                  float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector , upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector*width;
        m_up_vector = m_up_vector*height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;                                      vertices[0].u = uMin; vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;                     vertices[1].u = uMax; vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;       vertices[2].u = uMax; vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;                        vertices[3].u = uMin; vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;


    }

    RaySquareIntersection intersect(const Ray& ray) const {
        RaySquareIntersection intersection;
        intersection.intersectionExists = false;

        Vec3 origin=ray.origin();

        Vec3 direction = ray.direction();

        float D = dot(m_bottom_left,m_normal);

        double haut= D - dot(origin,m_normal); 

        double bas= dot(direction,m_normal);

        if(bas == 0.){
            return intersection;
        }

        double t=haut/bas;


        Vec3 point_intersection= origin+t*direction;

        Vec3 to_point = point_intersection - m_bottom_left;

        float u = dot(to_point,m_right_vector) / dot(m_right_vector,m_right_vector);
        float v = dot(to_point,m_up_vector) / dot(m_up_vector,m_up_vector);

        if (u >= 0 && u <= 1 && v >= 0 && v <= 1){
            
            intersection.intersectionExists=true;
            intersection.t=t;
            intersection.intersection=point_intersection;
            intersection.normal=m_normal;
            intersection.u=u;
            intersection.v=v;

        }
        
        return intersection;
    }




    // RaySquareIntersection intersect(const Ray &ray) const {
    //     RaySquareIntersection intersection;
    //     intersection.intersectionExists = false;

    //     Vec3 m_bottom_left = vertices[0].position;
    //     Vec3 m_right_vector = vertices[1].position - vertices[0].position;
    //     Vec3 m_up_vector = vertices[3].position - vertices[0].position;
    //     Vec3 m_normal = Vec3::cross(m_right_vector, m_up_vector);
    //     m_normal.normalize();
    //     Vec3 origine=ray.origin();
    //     Vec3 direction=ray.direction();
    //     float D= dot(m_bottom_left,m_normal);
    //     if(dot(direction,m_normal)==0){
    //         intersection.intersectionExists = false;
    //     }
    //     else{
    //         float t=(D-dot(origine,m_normal))/dot(direction,m_normal);            
    //         if (t>0){
    //             Vec3 intersection_point = origine + t * direction;
    //             Vec3 p = intersection_point - m_bottom_left;
    //             float u = dot(p, m_right_vector) / dot(m_right_vector, m_right_vector);
    //             float v = dot(p, m_up_vector) / dot(m_up_vector, m_up_vector);

    //             if (0 <= u && u <= 1 && 0 <= v && v <= 1) {
    //                 intersection.intersectionExists = true;
    //                 intersection.t=t;
    //                 intersection.intersection=origine+t*direction;
    //                 intersection.normal = m_normal;
    //             }
    //             else{
    //                 intersection.intersectionExists = false;
    //             }
    //         }
    //         else{
    //             intersection.intersectionExists=false;
    //         }
    //     }

    //     return intersection;
    // }




    void translate(Vec3 const & translation){
        Mesh::translate(translation);  
        m_bottom_left += translation;  
    }

     void scale(Vec3 const & scale)  {
        Mesh::scale(scale);  
        Mat3 scale_matrix(scale[0], 0., 0.,
                          0., scale[1], 0.,
                          0., 0., scale[2]);
        m_bottom_left = scale_matrix * m_bottom_left;
        m_right_vector = scale_matrix * m_right_vector;
        m_up_vector = scale_matrix * m_up_vector;
        m_normal = scale_matrix * m_normal;
        m_normal.normalize();  
    }

    // Redéfinition de la rotation autour de l'axe X
    void rotate_x(float angle)  {
        Mesh::rotate_x(angle);
        float x_angle = angle * M_PI / 180.;
        Mat3 x_rotation(1., 0., 0.,
                        0., cos(x_angle), -sin(x_angle),
                        0., sin(x_angle), cos(x_angle));
        apply_rotation(x_rotation);
    }

    // Redéfinition de la rotation autour de l'axe Y
    void rotate_y(float angle)  {
        Mesh::rotate_y(angle);
        float y_angle = angle * M_PI / 180.;
        Mat3 y_rotation(cos(y_angle), 0., sin(y_angle),
                        0., 1., 0.,
                        -sin(y_angle), 0., cos(y_angle));
        apply_rotation(y_rotation);
    }

    // Redéfinition de la rotation autour de l'axe Z
    void rotate_z(float angle)  {
        Mesh::rotate_z(angle);
        float z_angle = angle * M_PI / 180.;
        Mat3 z_rotation(cos(z_angle), -sin(z_angle), 0.,
                        sin(z_angle), cos(z_angle), 0.,
                        0., 0., 1.);
        apply_rotation(z_rotation);
    }

    void apply_rotation(Mat3 rotation_matrix) {
        m_bottom_left = rotation_matrix * m_bottom_left;
        m_right_vector = rotation_matrix * m_right_vector;
        m_up_vector = rotation_matrix * m_up_vector;
        m_normal = rotation_matrix * m_normal;
        m_normal.normalize();
    }



};
#endif // SQUARE_H
