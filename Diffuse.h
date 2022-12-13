#pragma once

class Diffuse: public ObjectMaterial {
public:
    Diffuse() = default;
    Diffuse( const float3& color ): color( color ) {}

    virtual MaterialType getFlag() const override {
        return MaterialType::DIFFUSE;
    }

    virtual float3 GetColor( Ray& ray_in ) const override {
        return color;
    }

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const override {
        ray_out = Ray( I, DiffuseReflection( N ) );
        return false;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        return new float[3] { color.x, color.y, color.z };
    }

private:
    float3 color;
};