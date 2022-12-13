#pragma once

class Mirror: public ObjectMaterial {
public:
    Mirror() = default;
    Mirror( const float3& color ): color( color ) {}

    virtual MaterialType getFlag() const override {
        return MaterialType::SPECULAR;
    }

    virtual float3 GetColor( Ray& ray_in ) const override {
        return color;
    }

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const override {
        ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
        return true;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        return new float[3] { color.x, color.y, color.z };
    }

private:
    float3 color;
};