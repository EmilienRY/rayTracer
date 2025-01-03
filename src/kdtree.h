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

// Boîte englobante
struct AABB {
    Vec3 min;
    Vec3 max;

    bool intersect(const Ray& ray) const {
        Vec3 origin =ray.origin();
        Vec3 direction =ray.direction();
        float tmin = (min[0] - origin[0]) / direction[0];
        float tmax = (max[0] - origin[0]) / direction[0];

        if (tmin > tmax) std::swap(tmin, tmax);

        float tymin = (min[1] - origin[1]) / direction[1];
        float tymax = (max[1] - origin[1]) / direction[1];
 
        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax)) 
            return false;

        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;

        float tzmin = (min[2] - origin[2]) / direction[2];
        float tzmax = (max[2] - origin[2]) / direction[2];
        
        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        return true;
    }
};

// Nœud du k-d tree
struct KDNode {
    AABB bbox;        // Boîte englobante
    int splitAxis;    // Axe de division (0=x, 1=y, 2=z)
    float splitPos;   // Position du plan de division
    KDNode* left;     // Enfant gauche
    KDNode* right;    // Enfant droit
    std::vector<Triangle> triangles; // Triangles si feuille
    bool isLeaf;      // Est-ce une feuille ?

    KDNode() : left(nullptr), right(nullptr), isLeaf(false) {}
};

class KDTree {
public:
    KDTree(const std::vector<Triangle>& triangles) {
        std::vector<Triangle> tris = triangles;
        root = build(tris, 0);
    }

    ~KDTree() {
        delete root;
    }

    bool intersect(const Ray& ray, RayTriangleIntersection& hitInfo) const {
        return intersectNode(root, ray, hitInfo);
    }

private:
    KDNode* root;

    float findBestSplitPos(const std::vector<Triangle>& triangles, int axis) {
        std::vector<float> positions;
        for (const auto& tri : triangles) {
            positions.push_back(tri.centroid()[axis]);
        }
        std::sort(positions.begin(), positions.end());
        return positions[positions.size()/2];  // médiane
    }

    KDNode* build(std::vector<Triangle>& triangles, int depth) {
        KDNode* node = new KDNode();
        node->bbox = calculateAABB(triangles); // Calcule la boîte englobante

        // Condition d'arrêt : si le nombre de triangles est petit ou la profondeur maximale est atteinte
        if (triangles.size() <= 5 || depth > 20) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        // Choisir l'axe de séparation (X, Y, Z) en fonction de la profondeur
        int axis = depth % 3;
        float splitPos = findBestSplitPos(triangles, axis);

        // Partitionner les triangles de chaque côté du plan
        std::vector<Triangle> leftTriangles, rightTriangles;
        for (auto& tri : triangles) {
            float vmin = std::min({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});
            float vmax = std::max({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});

            // Ajouter correctement les triangles aux deux côtés s'ils traversent le plan
            if (vmin <= splitPos && vmax >= splitPos) {
                leftTriangles.push_back(tri);
                rightTriangles.push_back(tri);
            } else if (vmin <= splitPos) {
                leftTriangles.push_back(tri);
            } else if (vmax >= splitPos) {
                rightTriangles.push_back(tri);
            }
        }

        // Éviter une récursion infinie si les sous-ensembles sont identiques ou vides
        if (leftTriangles.empty() || rightTriangles.empty()) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        // Enregistrer les informations de la séparation
        node->splitAxis = axis;
        node->splitPos = splitPos;

        // Récursion pour les sous-arbres gauche et droit
        node->left = build(leftTriangles, depth + 1);
        node->right = build(rightTriangles, depth + 1);

        return node;
    }



    bool intersectNode(KDNode* node, const Ray& ray, RayTriangleIntersection& hitInfo) const {
        if (!node->bbox.intersect(ray)) return false;
        
        if (node->isLeaf) {
            bool hit = false;
            float closest_t = FLT_MAX;
            
            for (const auto& tri : node->triangles) {
                RayTriangleIntersection tempHit = tri.getIntersection(ray);
                if (tempHit.intersectionExists && tempHit.t < closest_t) {
                    closest_t = tempHit.t;
                    hitInfo = tempHit;
                    hit = true;
                }
            }
            return hit;
        }
        
        RayTriangleIntersection leftHit, rightHit;
        bool hitLeft = intersectNode(node->left, ray, leftHit);
        bool hitRight = intersectNode(node->right, ray, rightHit);
        
        if (!hitLeft && !hitRight) return false;
        if (!hitLeft) { hitInfo = rightHit; return true; }
        if (!hitRight) { hitInfo = leftHit; return true; }
        
        hitInfo = (leftHit.t < rightHit.t) ? leftHit : rightHit;
        return true;
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