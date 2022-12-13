#pragma once

class Light: public ObjectMaterial {
public:
    Light() = default;
    Light( const float3& color, const float3& direction = float3( 0 ) ): color( color ), direction( direction ) {}

    virtual MaterialType getFlag() const override {
        return MaterialType::LIGHT;
    }

    virtual float3 GetColor( Ray& ray_in ) const override {
        return color;
    }

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const override {
        return false;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        return new float[3] { clamp( color.x, 0.0f, 1.0f ), clamp( color.y, 0.0f, 1.0f ), clamp( color.z, 0.0f, 1.0f ) };
    }

private:
    float3 color;
    float3 direction;
};