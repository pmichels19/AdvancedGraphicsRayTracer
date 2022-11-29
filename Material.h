#pragma once

class Material {
public:
    Material() = default;
    Material( float3 color, float diffuse, float n = 0 ) {
        this->color = color;

        // clamp diffuse in the interval [0, 1]
        this->diffuse = clamp(diffuse, 0.0f, 1.0f);
        this->specular = 1.0f - this->diffuse;

        // only accept refractive indices higher than 1, gots to keep nature real
        if ( n > 1 )  {
            this->n = n;
        }
    }

    virtual float3 GetColor( float3 I ) {
        return color;
    }

    float3 color;
    // standard material stuff
    float diffuse;
    float specular;
    // dielectric stuff - will override diffuse and specular
    float n = 0;
};
