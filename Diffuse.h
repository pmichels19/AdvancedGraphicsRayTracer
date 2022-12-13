#pragma once

class Diffuse: public ObjectMaterial {
public:
    Diffuse() = default;
    Diffuse( const float3& color ): color( color ) {}

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        float3 R = DiffuseReflection( N );
        ray_out = Ray( I, R );

        float3 BRDF = attenuation * INVPI;
        float toEi = dot( N, R );
        attenuation = PI * 2.0f * BRDF * toEi;

        return true;
    }

    virtual float3 GetColor( Ray& ray_in ) const override {
        return color;
    }

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const override {
        ray_out = Ray( I, DiffuseReflection( N ) );
        return false;
    }

    virtual MaterialType getFlag() const override {
        return MaterialType::DIFFUSE;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        return new float[3] { color.x, color.y, color.z };
    }

private:
    float3 color;
};