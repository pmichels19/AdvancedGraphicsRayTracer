#pragma once

class Material {
public:
    Material( float3 color, float diffuse ) {
        this->color = color;

        if ( diffuse >= 0 - FLT_EPSILON && diffuse <= 1 + FLT_EPSILON ) {
            this->diffuse = diffuse;
        } else if ( diffuse < 0  ) {
            this->diffuse = 0;
        } else {
            this->diffuse = 1;
        }

        this->specular = 1.0f - this->diffuse;
    }

    float3 color;

    float diffuse;
    float specular;
};
