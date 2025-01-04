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

    Vec3 size() const {
        return max - min;
    }

    float surfaceArea() const {
        Vec3 d = size();
        return 2.0f * (d[0] * d[1] + d[1] * d[2] + d[2] * d[0]);
    }

    bool intersect(const Ray& ray) const {
        Vec3 invDir = Vec3(1.0f/ray.direction()[0], 
                        1.0f/ray.direction()[1], 
                        1.0f/ray.direction()[2]);

        Vec3 origin=ray.origin();

        Vec3 t0 = Vec3(
            (min[0] - origin[0]) * invDir[0],
            (min[1] - origin[1]) * invDir[1],
            (min[2] - origin[2]) * invDir[2]
        );
        
        Vec3 t1 = Vec3(
            (max[0] - origin[0]) * invDir[0],
            (max[1] - origin[1]) * invDir[1],
            (max[2] - origin[2]) * invDir[2]
        );
        
        Vec3 tmin = Vec3::min(t0, t1);
        Vec3 tmax = Vec3::max(t0, t1);
        
        float tenter = std::max(std::max(tmin[0], tmin[1]), tmin[2]);
        float texit = std::min(std::min(tmax[0], tmax[1]), tmax[2]);

        return tenter <= texit && texit > 0.0f; 

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
    static const int SAH_THRESHOLD = 128;

    struct SplitInfo {
        float cost;
        float position;
        int leftCount;
        int rightCount;
    };



    float findSplitPos( std::vector<Triangle>& triangles, const AABB& bbox, int axis, bool useSAH) {
        if (!useSAH) {
            // Méthode rapide : moyenne entre le min et max sur l'axe
            float min = std::numeric_limits<float>::max();
            float max = std::numeric_limits<float>::lowest();
            
            for ( auto& tri : triangles) {
                for (int i = 0; i < 3; i++) {
                    float val = tri.getM_c(i)[axis];
                    min = std::min(min, val);
                    max = std::max(max, val);
                }
            }
            return (min + max) * 0.5f;
        }
        
        // Version simplifiée de SAH avec moins de bins
        const int NUM_BINS = 8;  // Réduit de 32 à 8 pour plus de rapidité
        std::vector<float> positions;
        positions.reserve(triangles.size() * 2);
        
        for ( auto& tri : triangles) {
            float min = std::min({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});
            float max = std::max({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});
            positions.push_back(min);
            positions.push_back(max);
        }
        
        float minPos = bbox.min[axis];
        float maxPos = bbox.max[axis];
        float bestCost = std::numeric_limits<float>::max();
        float bestPos = (minPos + maxPos) * 0.5f;
        
        // Évaluer seulement quelques positions uniformément réparties
        for (int i = 1; i < NUM_BINS; i++) {
            float splitPos = minPos + (maxPos - minPos) * (float)i / NUM_BINS;
            
            int leftCount = 0, rightCount = 0;
            for (const auto& tri : triangles) {
                float centroid = tri.centroid()[axis];
                if (centroid <= splitPos) leftCount++;
                else rightCount++;
            }
            
            float cost = leftCount + rightCount;  // Coût simplifié
            if (cost < bestCost) {
                bestCost = cost;
                bestPos = splitPos;
            }
        }
        
        return bestPos;
    }

    KDNode* build(std::vector<Triangle>& triangles, int depth) {
        KDNode* node = new KDNode();
        node->bbox = calculateAABB(triangles);

        // Critères d'arrêt ajustés
        const int MAX_TRIANGLES = 12;  // Augmenté légèrement
        const int MAX_DEPTH = 18;
        
        if (triangles.size() <= MAX_TRIANGLES || depth > MAX_DEPTH) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        // Choisir l'axe avec la plus grande étendue
        Vec3 extent = node->bbox.size();
        int axis = 0;
        if (extent[1] > extent[axis]) axis = 1;
        if (extent[2] > extent[axis]) axis = 2;

        // Utiliser SAH seulement pour les grands ensembles de triangles
        bool useSAH = triangles.size() > SAH_THRESHOLD;
        float splitPos = findSplitPos(triangles, node->bbox, axis, useSAH);

        // Partitionner les triangles avec une approche simplifiée
        std::vector<Triangle> leftTriangles, rightTriangles;
        for (const auto& tri : triangles) {
            float centroid = tri.centroid()[axis];
            if (centroid < splitPos) {
                leftTriangles.push_back(tri);
            }
            if (centroid > splitPos) {
                rightTriangles.push_back(tri);
            }
            // Si le triangle chevauche, ajoute-le aux deux côtés
            if (centroid == splitPos) {
                leftTriangles.push_back(tri);
                rightTriangles.push_back(tri);
            }
        }

        // Gérer les cas dégénérés
        if (leftTriangles.empty() || rightTriangles.empty()) {
            size_t mid = triangles.size() / 2;
            leftTriangles.assign(triangles.begin(), triangles.begin() + mid);
            rightTriangles.assign(triangles.begin() + mid, triangles.end());
        }

        node->splitAxis = axis;
        node->splitPos = splitPos;

        node->left = build(leftTriangles, depth + 1);
        node->right = build(rightTriangles, depth + 1);

        return node;
    }



    bool intersectNode(KDNode* node, const Ray& ray, RayTriangleIntersection& hitInfo) const {
        if (!node->bbox.intersect(ray)) return false;
        
        if (node->isLeaf) {
            bool hit = false;
            float closest_t = hitInfo.intersectionExists ? hitInfo.t : FLT_MAX;
            
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
        
        // Déterminer l'ordre de traversée basé sur la direction du rayon
        KDNode *first = node->left, *second = node->right;
        if (ray.direction()[node->splitAxis] > 0) {
            std::swap(first, second);
        }

        float splitDist = (node->splitPos - ray.origin()[node->splitAxis]) / 
                  (ray.direction()[node->splitAxis] + std::numeric_limits<float>::epsilon());

        
        // float splitDist = (node->splitPos - ray.origin()[node->splitAxis]) / 
        //                  ray.direction()[node->splitAxis];
        
        bool hit = false;
        
        // Traverser le premier nœud
        hit = intersectNode(first, ray, hitInfo);
        
        // Traverser le second nœud seulement si nécessaire
        if (!hit || (splitDist > 0 && (!hitInfo.intersectionExists || splitDist < hitInfo.t))) {
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