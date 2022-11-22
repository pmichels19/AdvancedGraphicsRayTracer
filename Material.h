#pragma once

class Material {
public:
    Material( float3 color, float diffuse, float specular ) {
        this->color = color;

        this->diffuse = diffuse;
        this->specular = specular;
    }

    float3 color;

    float diffuse;
    float specular;
};
