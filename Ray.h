#pragma once

#define PACKET_TRAVERSAL false
#define PACKET_SIZE 64
#define SQRT_PACKET_SIZE 8

class Ray {
public:
    Ray() = default;
    Ray( float3 origin, float3 direction, float distance = 1e34f ) {
        O = origin, D = direction, t = distance;
        // calculate reciprocal ray direction for triangles and AABBs
        rD = float3( 1 / D.x, 1 / D.y, 1 / D.z );
#ifdef SPEEDTRIX
        d0 = d1 = d2 = 0;
#endif
    }
    float3 IntersectionPoint() { return O + t * D; }
    // ray data
#ifndef SPEEDTRIX
    float3 O, D, rD;
#else
    union { struct { float3 O; float d0; }; __m128 O4; };
    union { struct { float3 D; float d1; }; __m128 D4; };
    union { struct { float3 rD; float d2; }; __m128 rD4; };
#endif
    float t = 1e34f;
    int objIdx = -1;
    bool inside = false; // true when in medium
    float u;
    float v;
};

class RayPacket {
public:
    // origins
    float3 O[PACKET_SIZE];
    // directions
    float3 D[PACKET_SIZE];
    // distances
    float t[PACKET_SIZE];
    // barycentric coordinates
    float u[PACKET_SIZE];
    float v[PACKET_SIZE];
    // object indices
    int objIdx[PACKET_SIZE];
    // the first ray that intersects an aabb
    int firstActive;

    RayPacket() {
        for ( int i = 0; i < PACKET_SIZE; i++ ) {
            t[i] = 1e34f;
            objIdx[i] = -1;
        }
    }

    Ray GetRay( int idx ) {
        Ray r = Ray( O[idx], D[idx], t[idx] );
        r.u = u[idx];
        r.v = v[idx];
        r.objIdx = objIdx[idx];
        return r;
    }
};