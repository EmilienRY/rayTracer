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


    bool intersect(const Ray& ray) const {
        const float EPSILON = 1e-7f;
        Vec3 invDir = Vec3(
            (std::abs(ray.direction()[0]) > EPSILON) ? 1.0f / ray.direction()[0] : std::numeric_limits<float>::infinity(),
            (std::abs(ray.direction()[1]) > EPSILON) ? 1.0f / ray.direction()[1] : std::numeric_limits<float>::infinity(),
            (std::abs(ray.direction()[2]) > EPSILON) ? 1.0f / ray.direction()[2] : std::numeric_limits<float>::infinity()
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

        return tmax > std::max(0.0f, tmin - EPSILON);
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

    bool intersect(const Ray& ray, RayTriangleIntersection& hitInfo) const {
        return intersectNode(root, ray, hitInfo);
    }

private:
    KDNode* root;
    static const int SAH_THRESHOLD = 128;


    KDNode* build(std::vector<Triangle>& triangles, int depth) {
        KDNode* node = new KDNode();
        node->bbox = calculateAABB(triangles);

        const int MAX_TRIANGLES = 8;
        const int MAX_DEPTH = 25;

        // Critère d'arrêt
        if (triangles.size() <= MAX_TRIANGLES || depth > MAX_DEPTH) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        // Axe avec la plus grande étendue
        Vec3 extent = node->bbox.size();
        int axis = (extent[1] > extent[0]) ? 1 : 0;
        axis = (extent[2] > extent[axis]) ? 2 : axis;

        // Trouver une position de split
        std::vector<float> centroids;
        for (const auto& tri : triangles) {
            centroids.push_back(tri.centroid()[axis]);
        }
        std::nth_element(centroids.begin(), centroids.begin() + centroids.size() / 2, centroids.end());


        float splitPos = centroids[centroids.size() / 2];


        // Éviter un mauvais split
        if (splitPos <= node->bbox.min[axis] || splitPos >= node->bbox.max[axis]) {
            node->isLeaf = true;
            node->triangles = triangles;
            return node;
        }

        std::vector<Triangle> leftTriangles, rightTriangles;
        for ( auto& tri : triangles) {
            float minCoord = std::min({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});
            float maxCoord = std::max({tri.getM_c(0)[axis], tri.getM_c(1)[axis], tri.getM_c(2)[axis]});

            float OVERLAP_EPSILON = 1e-5f;
            if (maxCoord <= splitPos + OVERLAP_EPSILON) {
                leftTriangles.push_back(tri);
            } else if (minCoord >= splitPos - OVERLAP_EPSILON) {
                rightTriangles.push_back(tri);
            } else {
                // Triangle traversant le plan
                leftTriangles.push_back(tri);
                rightTriangles.push_back(tri);
            }
        }

        // Éviter une récursion infinie
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
        if (!node->bbox.intersect(ray)) return false;
        
        if (node->isLeaf) {
            bool hit = false;
            float closest_t = hitInfo.intersectionExists ? hitInfo.t : FLT_MAX;
            const float INTERSECTION_EPSILON = 1e-6f;
            
            for (const auto& tri : node->triangles) {
                RayTriangleIntersection tempHit = tri.getIntersection(ray);
                // Ajouter une marge de sécurité pour la comparaison
                if (tempHit.intersectionExists && tempHit.t < closest_t - INTERSECTION_EPSILON) {
                    // Vérifier que l'intersection est valide
                    if (tempHit.t > INTERSECTION_EPSILON) {
                        closest_t = tempHit.t;
                        hitInfo = tempHit;
                        hit = true;
                    }
                }
            }
            return hit;
        }

        float dirComponent = ray.direction()[node->splitAxis];
        const float DIR_EPSILON = 1e-10f;
        
        if (std::abs(dirComponent) < DIR_EPSILON) {
            // Rayon parallèle au plan de séparation
            if (ray.origin()[node->splitAxis] <= node->splitPos) {
                return intersectNode(node->left, ray, hitInfo);
            } else {
                return intersectNode(node->right, ray, hitInfo);
            }
        }

        KDNode *first = (ray.origin()[node->splitAxis] <= node->splitPos) ? node->left : node->right;
        KDNode *second = (ray.origin()[node->splitAxis] <= node->splitPos) ? node->right : node->left;
        
        float splitDist = (node->splitPos - ray.origin()[node->splitAxis]) / dirComponent;
        bool hit = intersectNode(first, ray, hitInfo);
        
        // Ajouter une marge de tolérance pour la traversée du second nœud
        if (!hit || (splitDist > -DIR_EPSILON && (!hitInfo.intersectionExists || splitDist < hitInfo.t + DIR_EPSILON))) {
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