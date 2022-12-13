#pragma once

class Light: public ObjectMaterial {
public:
    Light() = default;
    Light( const float3& color, const float3& direction = float3( 0 ) ): color( color ), direction( direction ) {}
    
    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        if ( sqrLength( direction ) < FLT_EPSILON || dot( direction, N ) > 0 ) {
            attenuation = color;
        } else {
            attenuation = float3( 0 );
        }
        return false;
    }

    virtual MaterialType getFlag() const override {
        return MaterialType::LIGHT;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        return new float[3] { clamp( color.x, 0.0f, 1.0f ), clamp( color.y, 0.0f, 1.0f ), clamp( color.z, 0.0f, 1.0f ) };
    }

private:
    float3 color;
    float3 direction;
};