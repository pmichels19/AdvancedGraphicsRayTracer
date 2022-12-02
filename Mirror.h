#pragma once

class Mirror: public ObjectMaterial {
public:
    Mirror() = default;
    Mirror( const float3& color ): color( color ) {}

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        attenuation = color;
        ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );

        return true;
    }

private:
    float3 color;
};