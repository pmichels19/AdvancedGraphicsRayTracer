#pragma once

#define BIN_COUNT 8

struct BVHNode {
    float3 aabbMin;
    float3 aabbMax;
    uint leftFirst;
    uint primitiveCount;

    bool isLeaf() {
        return primitiveCount > 0;
    }
};

struct BVHBin {
    aabb bounds;
    int primitiveCount = 0;
};