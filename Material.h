#pragma once

class Material {
public:
    Material( float3 color, float diffuse ) {
        this->color = color;

        // clamp diffuse in the interval [0, 1]
        this->diffuse = fminf( 1, fmaxf( 0, diffuse ) );
        this->specular = 1.0f - this->diffuse;
    }

    float3 color;

    float diffuse;
    float specular;
};
