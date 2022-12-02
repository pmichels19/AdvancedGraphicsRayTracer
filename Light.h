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

private:
    float3 color;
    float3 direction;
};