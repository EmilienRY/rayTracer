#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <GL/glut.h>
#include <cmath>
#include <random>



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

    float focusDistance = 10.0f; 
    float aperture = 0.1f;       
    int nbSamples = 10;     
    std::mt19937 gen;     
    std::uniform_real_distribution<float> dist; 

public:


    Scene() {}

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


    bool ombreDur(Vec3 normal,Vec3 pointIntersection,Vec3 lightDir, Vec3 lightPos){
            Ray shadowRay(pointIntersection, lightDir);
            float maxDistanceToLight = (lightPos - pointIntersection).length();
            RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

            if (shadowIntersection.intersectionExists) {
                if ((shadowIntersection.typeOfIntersectedObject == 2 && 
                    shadowIntersection.raySphereIntersection.t > 0.01 && 
                    shadowIntersection.raySphereIntersection.t < maxDistanceToLight) ||
                    (shadowIntersection.typeOfIntersectedObject == 1 && 
                    shadowIntersection.raySquareIntersection.t > 0.01 && 
                    shadowIntersection.raySquareIntersection.t < maxDistanceToLight) ||
                    (shadowIntersection.typeOfIntersectedObject == 3 && 
                    shadowIntersection.rayMeshIntersection.t > 0.001 && 
                    shadowIntersection.rayMeshIntersection.t < maxDistanceToLight + 0.001)) {
                    return true;
                }
            }
            return false;
    }

    float ombreDouce(Vec3 normal,Vec3 pointIntersection, Vec3 lightPos,int nbEchan){
            int touche = 0;

            for (int i = 0; i < nbEchan; i++) {
                Vec3 lightEchanPos = randomPointDansCarre(lightPos, lights[0].radius);
                Vec3 lightEchanDir = (lightEchanPos - pointIntersection);
                lightEchanDir.normalize();
                float maxDistanceToLight = (lightEchanPos - pointIntersection).length();
                Ray shadowRay(pointIntersection + normal * 0.001f, lightEchanDir);

                RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

                if (shadowIntersection.intersectionExists) {
                    if ((shadowIntersection.typeOfIntersectedObject == 2 && 
                        shadowIntersection.raySphereIntersection.t > 0.01 && 
                        shadowIntersection.raySphereIntersection.t < maxDistanceToLight) ||
                        (shadowIntersection.typeOfIntersectedObject == 3 && 
                        shadowIntersection.rayMeshIntersection.t > 0.001 && 
                        shadowIntersection.rayMeshIntersection.t < maxDistanceToLight + 0.001)) {
                            touche++;
                    }
                }
            }

            float facteurOmbre = 1.0f - (float)touche / nbEchan;
            return facteurOmbre;
    }


    Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - 2*dot(v,n)*n;
    }

    Vec3 refract(const Vec3& v, const Vec3& n, float transparence) {

        Vec3 refraction =transparence*v+(transparence*dot(v,n)-sqrt(1-(transparence*transparence)*(1-pow(dot(v,n),2))))*n;
        return refraction;

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

        Vec3 newColor;
        if(material.type == Material_Mirror) {
            Vec3 bias = normal * 0.001f;
            Vec3 reflectedDir = reflect(ray.direction(), normal);
            reflectedDir.normalize();
            
            Ray reflectedRay(pointIntersection + bias, reflectedDir);
            newColor = rayTraceRecursive(reflectedRay, NRemainingBounces - 1);
            color += newColor;
        }
        else if(material.type==Material_Glass){

            float indRefraction=1.0008/material.transparency;
            Vec3 refraction =refract(ray.direction(),normal,indRefraction);
            Ray refractedRay(pointIntersection,refraction);
            newColor = rayTraceRecursive(refractedRay, NRemainingBounces-1);
            color+=newColor;

        }
        else{   


            Vec3 lightPos = lights[0].pos;
            Vec3 lightColor = lights[0].material;
            Vec3 lightDir = lightPos - pointIntersection;

            if (lightDir.length() > 0.0f) {
                lightDir.normalize();
            }
            float facteurOmbre=1.0;

            if(ombreDur(normal,pointIntersection,lightDir,lightPos)){
                return Vec3(0,0,0);
            }

            // facteurOmbre=ombreDouce(normal,pointIntersection,lightPos,10);


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


            color = ((ambient + diffuse + specular )*facteurOmbre);

        }


        return color;
    }


    Vec3 rayTrace(Ray const & rayStart) {
        return rayTraceRecursive(rayStart, 5); 
    }



    Ray generateRay(const Ray& cameraRay) {
        Vec3 pointFocus = cameraRay.origin() + cameraRay.direction() * focusDistance;

        float offsetX = dist(gen);
        float offsetY = dist(gen);

        Vec3 nouvPointDep = cameraRay.origin() + Vec3(offsetX, offsetY, 0);

        Vec3 newDirection = (pointFocus - nouvPointDep);
        newDirection.normalize();
        return Ray(nouvPointDep, newDirection);
    }


    void initializeRandomGenerator() {
        std::random_device rd;
        gen = std::mt19937(rd()); 
        dist = std::uniform_real_distribution<float>(-aperture / 2.0f, aperture / 2.0f);  
    }

    // Vec3 rayTrace(const Ray& rayStart) {
    //     Vec3 finalColor(0.0f, 0.0f, 0.0f);

    //     for (int i = 0; i < nbSamples; ++i) {
    //         Ray nouvRay = generateRay(rayStart);
    //         finalColor += rayTraceRecursive(nouvRay, 5); 
    //     }

    //     finalColor = finalColor/nbSamples; 

    //     return finalColor;
    // }


    void setup_depth_of_field(float focusDist, float apert, int samples) {
        focusDistance = focusDist;
        aperture = apert;
        nbSamples = samples;
    }

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
        setup_depth_of_field(2.0f, 0.f, 1);
        initializeRandomGenerator();
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
            // m.loadOFF("./mesh/dragon.off");
            // m.scale(Vec3(5.5,5.5,5.5)); 
            m.loadOFF("./mesh/sphere.off");
            m.scale(Vec3(0.9,0.9,0.9)); 
            m.translate(Vec3(-1.0, -1.25, 1)); 
            m.build_arrays();
            // m.material.type=Material_Mirror;
            m.material.diffuse_material = Vec3( 1.,0.,0. );
            m.material.specular_material = Vec3( 1.,0.,0. );
            m.material.shininess = 16;
            m.material.transparency = 1.5;
            m.material.index_medium = 1.4;
            m.buildKDTree();


        }

    }


    void setup_rendu_raptor(){
        meshes.clear();
        spheres.clear(); 
        squares.clear();
        lights.clear();
        setup_depth_of_field(2.0f, 0.f, 10);
        initializeRandomGenerator();

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

 
        {

            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.1, -0.9, -1);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass; 
            s.material.diffuse_material = Vec3(0.8, 0.8, 0.8); 
            s.material.specular_material = Vec3(1., 1., 1.);
            s.material.shininess = 16;
            s.material.transparency = 1.5;
            s.material.index_medium = 1.4;

        }   

        {

            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.3, 1.3, -1.3);
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
            // m.loadOFF("./mesh/dragon.off");
            // m.scale(Vec3(5.5,5.5,5.5)); 
            m.loadOFF("./mesh/raptor.off");
            m.scale(Vec3(1.6,1.6,1.6)); 
            m.rotate_y(90.f);
            m.translate(Vec3(-1., -1.3, -0.5)); 

            m.build_arrays();
            // m.material.type=Material_Mirror;
            m.material.diffuse_material = Vec3( 1.,0.,0. );
            m.material.specular_material = Vec3( 1.,0.,0. );
            m.material.shininess = 16;
            m.material.transparency = 1.5;
            m.material.index_medium = 1.4;
            m.buildKDTree();


        }



        { //imported sphere
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            // m.loadOFF("./mesh/dragon.off");
            // m.scale(Vec3(5.5,5.5,5.5)); 
            m.loadOFF("./mesh/raptor.off");
            m.scale(Vec3(1.6,1.6,1.6)); 
            m.translate(Vec3(0.7, -1.3, 1)); 
            m.build_arrays();
            m.material.type=Material_Mirror;
            m.material.diffuse_material = Vec3( 1.,0.,0. );
            m.material.specular_material = Vec3( 1.,0.,0. );
            m.material.shininess = 16;
            m.material.transparency = 1.5;
            m.material.index_medium = 1.4;
            m.buildKDTree();


        }


        { //imported sphere
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            // m.loadOFF("./mesh/dragon.off");
            // m.scale(Vec3(5.5,5.5,5.5)); 
            m.loadOFF("./mesh/squirrel.off");
            m.scale(Vec3(0.1,0.1,0.1)); 
            m.translate(Vec3(-1., -1.4, 1)); 
            m.build_arrays();
            // m.material.type=Material_Mirror;
            m.material.diffuse_material = Vec3( 1.,0.,0. );
            m.material.specular_material = Vec3( 1.,0.,0. );
            m.material.shininess = 16;
            m.material.transparency = 1.5;
            m.material.index_medium = 1.4;
            m.buildKDTree();


        }

    }


};


#endif
