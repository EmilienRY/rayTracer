#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"

#include <fstream>
#include <iostream>

#include <GL/glut.h>


enum LightType {
    LightType_Spherical,
    LightType_Quad
};


struct Light {
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Mesh quad;

    float powerCorrection;

    Light() : powerCorrection(1.0) {}

};

struct RaySceneIntersection{
    bool intersectionExists;
    unsigned int typeOfIntersectedObject;
    unsigned int objectIndex;
    float t;
    RayTriangleIntersection rayMeshIntersection;
    RaySphereIntersection raySphereIntersection;
    RaySquareIntersection raySquareIntersection;
    RaySceneIntersection() : intersectionExists(false) , t(FLT_MAX) {}
};



class Scene {
    std::vector< Mesh > meshes;
    std::vector< Sphere > spheres;
    std::vector< Square > squares;
    std::vector< Light > lights;

public:


    Scene() {
    }

    void draw() {
        // iterer sur l'ensemble des objets, et faire leur rendu :
        for( unsigned int It = 0 ; It < meshes.size() ; ++It ) {
            Mesh const & mesh = meshes[It];
            mesh.draw();
        }
        for( unsigned int It = 0 ; It < spheres.size() ; ++It ) {
            Sphere const & sphere = spheres[It];
            sphere.draw();
        }
        for( unsigned int It = 0 ; It < squares.size() ; ++It ) {
            Square const & square = squares[It];
            square.draw();
        }
    }



    RaySceneIntersection computeIntersection(Ray const & ray) {
        RaySceneIntersection result;

        float tMinSphere=10000000.0;
        float tMinSquare=10000000.0;
        float tMinTriangle=10000000.0;

        RaySphereIntersection raySphereIntersectionMin= RaySphereIntersection();
        RaySquareIntersection raySquareIntersectionMin = RaySquareIntersection();
        RayTriangleIntersection rayTriangleIntersectionMin = RayTriangleIntersection();

        int indexItSphere=-1;
        int indexItSquare=-1;
        int indexItTriangle=-1;
        for( unsigned int It = 0 ; It < spheres.size() ; It++ ) {
            Sphere const & sphere = spheres[It];
            RaySphereIntersection interSphere = sphere.intersect(ray);
            if(interSphere.intersectionExists){
                if(interSphere.t<tMinSphere){
                    indexItSphere=It;
                    tMinSphere=interSphere.t;
                    raySphereIntersectionMin=interSphere;
                }
            }
        }


        for( unsigned int It = 0 ; It < squares.size() ; It++ ) {
            Square const & square = squares[It];
            RaySquareIntersection interSquare = square.intersect(ray);
            if(interSquare.intersectionExists){
                if(interSquare.t<tMinSquare){
                    if(dot(ray.direction(),interSquare.normal)>=0){
                        continue;
                    }
                    else{
                        indexItSquare=It;
                        tMinSquare=interSquare.t;
                        raySquareIntersectionMin=interSquare;
                    }
                }
            }
        }

        for( unsigned int It = 0 ; It < meshes.size() ; It++ ) {
            Mesh const & mesh = meshes[It];
            RayTriangleIntersection interMesh = mesh.intersect(ray);
            if(interMesh.intersectionExists){
                if(interMesh.t<tMinTriangle){
                    indexItTriangle=It;
                    tMinTriangle=interMesh.t;
                    rayTriangleIntersectionMin=interMesh;
                }
            }
        }

        if( indexItSphere != -1 || indexItSquare != -1 || indexItTriangle != -1 ){
            float tMin=std::min(tMinSphere,std::min(tMinSquare,tMinTriangle));
            
            if(tMin==tMinSphere){
                result.t=tMinSphere;
                result.objectIndex=indexItSphere;
                result.intersectionExists=true;
                result.typeOfIntersectedObject=2;
                result.raySphereIntersection=raySphereIntersectionMin;
            }
            else if(tMin==tMinSquare){
                result.t=tMinSquare;
                result.objectIndex=indexItSquare;
                result.intersectionExists=true;
                result.typeOfIntersectedObject=1;
                result.raySquareIntersection=raySquareIntersectionMin;
            }
            else if(tMin==tMinTriangle){
                result.t=tMinTriangle;
                result.objectIndex=indexItTriangle;
                result.intersectionExists=true;
                result.typeOfIntersectedObject=3;
                result.rayMeshIntersection=rayTriangleIntersectionMin;  
            }

        }else{
            result.intersectionExists=false;
        }

        return result;
    }

    Vec3 randomPointDansCarre(const Vec3 &center, float radius) {
        float offsetX = (2 * (rand() / (float)RAND_MAX) - 1) * radius;
        float offsetY = (2 * (rand() / (float)RAND_MAX) - 1) * radius;

        return center + Vec3(offsetX, offsetY, 0); 
    }




    Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - 2*dot(v,n)*n;
    }

    Vec3 refract(const Vec3& uv, const Vec3& n, double etai_over_etat) {
        auto cos_theta = std::fmin(dot(-1*uv, n), 1.0);
        Vec3 r_out_perp =  etai_over_etat * (uv + cos_theta*n);
        Vec3 r_out_parallel = -std::sqrt(std::fabs(1.0 - r_out_perp.squareLength())) * n;
        return r_out_perp + r_out_parallel;
    }

    Vec3 rayTraceRecursive(Ray ray, int NRemainingBounces) {
        Vec3 color=Vec3(0,0,0);


        if(NRemainingBounces == 0) {

            return Vec3(0,0,0);
        }

        RaySceneIntersection intersection = computeIntersection(ray);
        if(!intersection.intersectionExists) {
            return color;
        }

        Vec3 pointIntersection;
        Vec3 normal;
        Material material;

        // Get intersection information based on object type
        if(intersection.typeOfIntersectedObject == 2) {
            pointIntersection = intersection.raySphereIntersection.intersection;
            normal = intersection.raySphereIntersection.normal;
            material = spheres[intersection.objectIndex].material;
        }
        else if(intersection.typeOfIntersectedObject == 1) {
            pointIntersection = intersection.raySquareIntersection.intersection;
            normal = intersection.raySquareIntersection.normal;
            material = squares[intersection.objectIndex].material;
        }
        else if(intersection.typeOfIntersectedObject == 3) {
            pointIntersection = intersection.rayMeshIntersection.intersection;
            normal = intersection.rayMeshIntersection.normal;
            material = meshes[intersection.objectIndex].material;
        } 


        if(material.type == Material_Mirror) {
            Vec3 bias = normal * 0.001f;
            Vec3 reflectedDir = reflect(ray.direction(), normal);
            reflectedDir.normalize();
            
            Ray reflectedRay(pointIntersection + bias, reflectedDir);
            Vec3 refractedColor = rayTraceRecursive(reflectedRay, NRemainingBounces - 1);
            color += refractedColor;
        }
        else if(material.type==Material_Glass){
            float transparence=1.0008/material.transparency;
            Vec3 refraction =transparence*ray.direction()+(transparence*dot(ray.direction(),normal)-sqrt(1-(transparence*transparence)*(1-pow(dot(ray.direction(),normal),2))))*normal;
            Ray glassRay(pointIntersection,refraction);
            Vec3 refractedColor;
            refractedColor+= rayTraceRecursive(glassRay, NRemainingBounces-1);
            color=refractedColor;
        }
        else{   


            Vec3 lightPos = lights[0].pos;
            Vec3 lightColor = lights[0].material;
            Vec3 lightDir = lightPos - pointIntersection;
            if (lightDir.length() > 0.0f) {
                lightDir.normalize();
            }


            // Vec3 bias = normal * 0.001f;
            // Ray shadowRay(pointIntersection, lightDir);
            // float maxDistanceToLight = (lightPos - pointIntersection).length();
            // RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

            // if (shadowIntersection.intersectionExists) {
            //     if ((shadowIntersection.typeOfIntersectedObject == 2 && 
            //         shadowIntersection.raySphereIntersection.t > 0.01 && 
            //         shadowIntersection.raySphereIntersection.t < maxDistanceToLight) ||
            //         (shadowIntersection.typeOfIntersectedObject == 1 && 
            //         shadowIntersection.raySquareIntersection.t > 0.01 && 
            //         shadowIntersection.raySquareIntersection.t < maxDistanceToLight) ||
            //         (shadowIntersection.typeOfIntersectedObject == 3 && 
            //         shadowIntersection.rayMeshIntersection.t > 0.001 && 
            //         shadowIntersection.rayMeshIntersection.t < maxDistanceToLight + 0.001)) {
            //         return Vec3(0,0,0);
            //     }
            // }


            // Ambient
            Vec3 ambient = Vec3(material.ambient_material[0] * lightColor[0],
                            material.ambient_material[1] * lightColor[1],
                            material.ambient_material[2] * lightColor[2]);

            // Diffuse
            float diffuseFactor = std::max(dot(normal, lightDir), 0.0f);
            Vec3 diffuse = Vec3(material.diffuse_material[0] * lightColor[0] * diffuseFactor,
                            material.diffuse_material[1] * lightColor[1] * diffuseFactor,
                            material.diffuse_material[2] * lightColor[2] * diffuseFactor);

            // Specular
            Vec3 viewDir = -1*ray.direction();
            Vec3 reflectDir = 2 * dot(normal, lightDir) * normal - lightDir;
            float specFactor = std::pow(std::max(dot(viewDir, reflectDir), 0.0f), material.shininess);
            Vec3 specular(
                material.specular_material[0] * lightColor[0] * specFactor,
                material.specular_material[1] * lightColor[1] * specFactor,                
                material.specular_material[2] * lightColor[2] * specFactor
            );

            color+= ambient + diffuse + specular;


        }


        return color;


    }



    Vec3 rayTrace(Ray const & rayStart) {
        return rayTraceRecursive(rayStart, 200); 
    }





    // Vec3 rayTrace( Ray const & rayStart ) {
    //     //TODO appeler la fonction recursive
    //     Vec3 color;

    //     RaySceneIntersection raySceneIntersection = computeIntersection(rayStart);

    //     Vec3 pointIntersection;
    //     Vec3 normal;
    //     Material material;


    //     if(raySceneIntersection.intersectionExists){
    //         if(raySceneIntersection.typeOfIntersectedObject == 2 ){
    //             pointIntersection= raySceneIntersection.raySphereIntersection.intersection;
    //             normal=raySceneIntersection.raySphereIntersection.normal;
    //             material = spheres[raySceneIntersection.objectIndex].material;

    //         }
    //         else if(raySceneIntersection.typeOfIntersectedObject == 1){
    //             pointIntersection= raySceneIntersection.raySquareIntersection.intersection;
    //             normal=raySceneIntersection.raySquareIntersection.normal;
    //             material = squares[raySceneIntersection.objectIndex].material;
 
    //         }
    //         else if(raySceneIntersection.typeOfIntersectedObject == 3){
    //             pointIntersection= raySceneIntersection.rayMeshIntersection.intersection;
    //             normal=raySceneIntersection.rayMeshIntersection.normal;
    //             material = meshes[raySceneIntersection.objectIndex].material;
 
    //         }
    //     }
      


    //     Vec3 lightPos = lights[0].pos;
    //     Vec3 lightColor = lights[0].material;
    //     Vec3 lightDir = lightPos - pointIntersection;
    //     if (lightDir.length() > 0.0f) {
    //         lightDir.normalize();
    //     }

    //     Vec3 bias = normal * 0.001; // Évite les auto-intersections
    //     Ray rayOmbre = Ray(pointIntersection + bias, lightDir);

    //     float maxDistanceToLight = (lightPos - pointIntersection).length();

    //     RaySceneIntersection rayOmbreIntersection = computeIntersection(rayOmbre);

    //     if (rayOmbreIntersection.intersectionExists) {
    //         if ((rayOmbreIntersection.typeOfIntersectedObject == 2 && 
    //             rayOmbreIntersection.raySphereIntersection.t > 0.01 && 
    //             rayOmbreIntersection.raySphereIntersection.t < maxDistanceToLight) ||
    //             (rayOmbreIntersection.typeOfIntersectedObject == 1 && 
    //             rayOmbreIntersection.raySquareIntersection.t > 0.01 && 
    //             rayOmbreIntersection.raySquareIntersection.t < maxDistanceToLight) ||
    //             (rayOmbreIntersection.typeOfIntersectedObject == 3 && 
    //             rayOmbreIntersection.rayMeshIntersection.t > 0.001 && 
    //             rayOmbreIntersection.rayMeshIntersection.t < maxDistanceToLight + 0.001)) {
    //             return Vec3(0,0,0); 
    //         }
    //     }





        

    //     // int echan = 50;  
    //     // int touche = 0;

    //     // for (int i = 0; i < echan; i++) {
    //     //     Vec3 lightEchanPos = randomPointDansCarre(lightPos, lights[0].radius);
    //     //     Vec3 lightEchanDir = (lightEchanPos - pointIntersection);
    //     //     lightEchanDir.normalize();
    //     //     float maxDistanceToLight = (lightEchanPos - pointIntersection).length();

    //     //     Ray shadowRay(pointIntersection + normal * 0.00001f, lightEchanDir); 
    //     //     RaySceneIntersection ombreIntersection = computeIntersection(shadowRay);

    //     //     if (ombreIntersection.intersectionExists && ombreIntersection.raySphereIntersection.t > 0.01 
    //     //         && ombreIntersection.raySphereIntersection.t < maxDistanceToLight) {
    //     //         touche++;

    //     //     }
    //     // }

    //     // float facteurOmbre = 1.0f - (float)touche / echan;


    //     Vec3 ambient= Vec3(material.ambient_material[0] * lightColor[0],
    //                        material.ambient_material[1] * lightColor[1],
    //                        material.ambient_material[2] * lightColor[2]
    //                     );
                        


    //     float diffuseFactor = std::max(dot(normal, lightDir), 0.0f);

    //     double diffusex = material.diffuse_material[0] * lightColor[0] * diffuseFactor;
    //     double diffusey = material.diffuse_material[1] * lightColor[1] * diffuseFactor;
    //     double diffusez = material.diffuse_material[2] * lightColor[2] * diffuseFactor;

    //     Vec3 diffuse = Vec3(diffusex,diffusey,diffusez);



    //     Vec3 viewDir = -1*rayStart.direction();
    //     Vec3 reflectDir = 2 * dot(normal, lightDir) * normal - lightDir;
    //     float specFactor = std::pow(std::max(dot(viewDir, reflectDir), 0.0f), material.shininess);
    //     Vec3 specular(
    //         material.specular_material[0] * lightColor[0] * specFactor,
    //         material.specular_material[1] * lightColor[1] * specFactor,                
    //         material.specular_material[2] * lightColor[2] * specFactor
    //     );


    //     // return (diffuse+ambient+specular)*facteurOmbre;

    //     return (diffuse+ambient+specular);

    //     // return material.diffuse_material;
    // }

    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0. , 0. , 0.);
            s.m_radius = 1.0f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
    }

    void setup_single_square() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.8,0.8,0.8 );
            s.material.specular_material = Vec3( 0.8,0.8,0.8 );
            s.material.shininess = 20;
        }


        { 
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.,0.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }

    }

    void setup_cornell_box(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 1.5, 0.0 );
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        
        { //Back Wall
            squares.resize( squares.size() + 1 );

            Square & s = squares[squares.size() - 1];

            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.,0.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;


        }

        { //Left Wall

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;


        }

        { //Right Wall

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.0,1.0,0.0 );
            s.material.specular_material = Vec3( 0.0,1.0,0.0 );
            s.material.shininess = 16;

        }

        { //Floor

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,0.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;

        }
        { //Ceiling

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();

            s.material.diffuse_material = Vec3( 0.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;

        }

        { //Front Wall

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            // s.material.type = Material_Mirror; 

            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(180);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,0.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;

        }


        { //GLASS Sphere

            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.0, -1.25, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass; 
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.5;
            s.material.index_medium = 1.4;


        }


        { //MIRRORED Sphere

            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.0, -1.25, -0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Mirror; 
            s.material.diffuse_material = Vec3(0.8, 0.8, 0.8); 
            s.material.specular_material = Vec3(1., 1., 1.);
            s.material.shininess = 16;
            s.material.transparency = 1.5;
            s.material.index_medium = 1.4;

        }

        { //imported sphere
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF("./mesh/sphere.off");
            m.scale(Vec3(0.5,0.5,0.5)); 
            m.translate(Vec3(-1., -1.5, 1)); 
            m.build_arrays();
            //m.material.type=Material_Mirror;
            m.material.diffuse_material = Vec3( 1.,0.,0. );
            m.material.specular_material = Vec3( 1.,0.,0. );
            m.material.shininess = 16;
            m.material.transparency = 1.5;
            m.material.index_medium = 1.4;

        }

    }
};
            //m.scale(Vec3(1.2,1.2,1.2)); //Pour la sphère
            //m.rotate_x(-10); //Pour Suzanne aussi

#endif
