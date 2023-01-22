#pragma once

#define BIN_COUNT 32

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
    vector<uint> leftChildren;
    vector<uint> rightChildren;

    vector<uint> splitPrimitives() {
        vector<uint> intersection;
        set_intersection( leftChildren.begin(), leftChildren.end(), rightChildren.begin(), rightChildren.end(), back_inserter( intersection ) );
        return intersection;
    }

    float cost() {
        return leftChildren.size() * left.Area() + rightChildren.size() * right.Area();
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