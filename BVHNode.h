#pragma once

struct BVHNode {
    float3 aabbMin;
    float3 aabbMax;
    uint left;
    uint firstPrimitiveIdx;
    uint primitiveCount;

    bool isLeaf() {
        return primitiveCount > 0;
    }
};