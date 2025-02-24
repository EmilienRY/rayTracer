#ifndef KDTREE_H
#define KDTREE_H

#include "Vec3.h"
#include "Mesh.h"
#include <vector>
#include <string>
#include "Ray.h"
#include "Triangle.h"
#include "Material.h"
#include <GL/glut.h>
#include <cfloat>
#include <algorithm>
#include <limits>

struct AABB {
    Vec3 min;
    Vec3 max;

    Vec3 size() const {
        return max - min;
    }


    bool intersect(const Ray& ray) const {

        Vec3 invDir = Vec3(
            (std::abs(ray.direction()[0]) > 0) ? 1.0f / ray.direction()[0] : std::numeric_limits<float>::infinity(),
            (std::abs(ray.direction()[1]) > 0) ? 1.0f / ray.direction()[1] : std::numeric_limits<float>::infinity(),
            (std::abs(ray.direction()[2]) > 0) ? 1.0f / ray.direction()[2] : std::numeric_limits<float>::infinity()
        );

        Vec3 origin = ray.origin();
        float tmin = -std::numeric_limits<float>::infinity();
        float tmax = std::numeric_limits<float>::infinity();

        for (int i = 0; i < 3; ++i) {
            float t1 = (min[i] - origin[i]) * invDir[i];
            float t2 = (max[i] - origin[i]) * invDir[i];

            if (invDir[i] < 0.0f) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
        }

        return tmax > std::max(0.0f, tmin);
    }

};

struct KDNode {
    AABB boiteEnglo;        
    int splitAxis;    
    float splitPos;  
    KDNode* left;     
    KDNode* right;    
    std::vector<Triangle> triangles; 
    bool isLeaf;      

    KDNode() : left(nullptr), right(nullptr), isLeaf(false) {}
};

class KDTree {
public:
    KDTree(const std::vector<Triangle>& triangles) {
        std::vector<Triangle> tris = triangles;
        racine = build(tris, 0);
    }

    bool intersect(const Ray& ray, RayTriangleIntersection& hitInfo) const {
        return intersectNode(racine, ray, hitInfo);
    }

private:
    KDNode* racine;


    KDNode* build(std::vector<Triangle>& triangles, int depth) {
        KDNode* node = new KDNode();
        node->boiteEnglo = calculateAABB(triangles);

        const int minTriangles = 8;
        const int maxProf = 25;

        if (triangles.size() <= minTriangles || depth > maxProf) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        Vec3 sizeBoite = node->boiteEnglo.size();
        int axis = (sizeBoite[1] > sizeBoite[0]) ? 1 : 0;
        axis = (sizeBoite[2] > sizeBoite[axis]) ? 2 : axis;

        std::vector<float> listeCentreTriangles;
        for (Triangle & tri : triangles) {
            listeCentreTriangles.push_back(tri.centre()[axis]);
        }
        std::nth_element(listeCentreTriangles.begin(), listeCentreTriangles.begin() + listeCentreTriangles.size() / 2, listeCentreTriangles.end());


        float splitPos = listeCentreTriangles[listeCentreTriangles.size() / 2];


        if (splitPos <= node->boiteEnglo.min[axis] || splitPos >= node->boiteEnglo.max[axis]) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }


        std::vector<Triangle> leftTriangles, rightTriangles;
        for ( auto& tri : triangles) {
            float minCoord = std::min({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});
            float maxCoord = std::max({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});

            if (maxCoord <= splitPos ) {
                leftTriangles.push_back(tri);
            } else if (minCoord >= splitPos) {
                rightTriangles.push_back(tri);
            } else {
                leftTriangles.push_back(tri);
                rightTriangles.push_back(tri);
            }
        }

        if (leftTriangles.empty() || rightTriangles.empty() || 
            leftTriangles.size() == triangles.size() || 
            rightTriangles.size() == triangles.size()) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        node->splitAxis = axis;
        node->splitPos = splitPos;

        node->left = build(leftTriangles, depth + 1);
        node->right = build(rightTriangles, depth + 1);

        return node;
    }



    bool intersectNode(KDNode* node, const Ray& ray, RayTriangleIntersection& hitInfo) const {
        if (!node->boiteEnglo.intersect(ray)) return false;
        
        if (node->isLeaf) {
            bool hit = false;
            float closest_t = hitInfo.intersectionExists ? hitInfo.t : FLT_MAX;
            
            for (const Triangle & tri : node->triangles) {
                RayTriangleIntersection tempHit = tri.getIntersection(ray);
                if (tempHit.intersectionExists && tempHit.t < closest_t) {
              
                    closest_t = tempHit.t;
                    hitInfo = tempHit;
                    hit = true;
                    
                }
            }
            return hit;
        }

        float originComponent = ray.origin()[node->splitAxis];
        float dirComponent = ray.direction()[node->splitAxis];

        KDNode* first;
        KDNode* second;

        if (dirComponent > 0) {
            first = (originComponent <= node->splitPos) ? node->left : node->right;
            second = (originComponent <= node->splitPos) ? node->right : node->left;
        } else if (dirComponent < 0) {
            first = (originComponent >= node->splitPos) ? node->right : node->left;
            second = (originComponent >= node->splitPos) ? node->left : node->right;
        }
        else{
            first = node->left;
            second = node->right;    
        }

        bool hit = intersectNode(first, ray, hitInfo);

        float splitDist = (node->splitPos - originComponent) / dirComponent;
        
        if (splitDist >= 0 && (!hit || splitDist < hitInfo.t)) {
            hit |= intersectNode(second, ray, hitInfo);
        }

        return hit;
    }



    AABB calculateAABB( std::vector<Triangle>& triangles) const {
        Vec3 min = Vec3(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
        Vec3 max = Vec3(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest());

        for (auto& tri : triangles) {
            for (int i = 0; i < 3; i++) {
                min = Vec3::min(min, tri.getM_c(i));
                max = Vec3::max(max, tri.getM_c(i));
            }
        }

        return {min, max};
    }
};

#endif