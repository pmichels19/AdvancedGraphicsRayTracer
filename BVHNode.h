#pragma once

#define BIN_COUNT 16

struct BVHNode {
    float3 aabbMin;
    float3 aabbMax;
    uint leftFirst;
    uint primitiveCount;

    bool isLeaf() {
        return primitiveCount > 0;
    }
};

struct BVHSplit {
    aabb left;
    aabb right;
    vector<int> leftChildren;
    vector<int> rightChildren;

    float intersectionArea() {
        return left.Intersection(right).Area();
    }
};

struct BVHBin {
    aabb bounds;
#ifdef SPATIAL_SPLITS
    int entry = 0;
    int exit = 0;
#endif
    int primitiveCount = 0;
};