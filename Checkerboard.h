#pragma once

class Checkerboard: public ObjectMaterial {
public:
    Checkerboard() = default;
    Checkerboard( const float3& color1, const float3& color2, const float& diffuse ): color1( color1 ), color2( color2 ) {
        this->diffuse = clamp( diffuse, 0.0f, 1.0f );
        specular = 1.0f - this->diffuse;
    }

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        bool evenX = abs( ( (int) floor( I.x ) ) % 2 ) == 0;
        bool evenZ = abs( ( (int) floor( I.z ) ) % 2 ) == 0;
        if ( evenX == evenZ ) {
            attenuation = color1;
        } else {
            attenuation = color2;
        }

        if ( diffuse < FLT_EPSILON ) {
            // if diffuse is 0, do reflection
            ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
        } else if ( specular < FLT_EPSILON ) {
            // if specular is 0, do diffuse reflection
            float3 R = DiffuseReflection( N );
            ray_out = Ray( I, R );
            attenuation *= 2.0f * dot( N, R );
        } else {
            // if both are > 0 we pick one randomly
            if ( random_float( 0, 1 ) < specular ) {
                ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
            } else {
                float3 R = DiffuseReflection( N );
                ray_out = Ray( I, R );
                attenuation *= 2.0f * dot( N, R );
            }
        }

        return true;
    }

private:
    float3 color1;
    float3 color2;
    float diffuse;
    float specular;
};