#pragma once

class Checkerboard: public ObjectMaterial {
public:
    Checkerboard() = default;
    Checkerboard( const float3& color1, const float3& color2, const float& diffuse ): color1( color1 ), color2( color2 ) {
        this->diffuse = clamp( diffuse, 0.0f, 1.0f );
        specular = 1.0f - this->diffuse;
    }

    virtual MaterialType getFlag() const override {
        if ( diffuse < FLT_EPSILON ) {
            return MaterialType::SPECULAR;
        }

        if ( specular < FLT_EPSILON ) {
            return MaterialType::DIFFUSE;
        }

        return MaterialType::MIX;
    }

    virtual float3 GetColor( Ray& ray_in ) const override {
        float3 I = ray_in.IntersectionPoint();
        bool evenX = abs( ( (int) floor( I.x ) ) % 2 ) == 0;
        bool evenZ = abs( ( (int) floor( I.z ) ) % 2 ) == 0;
        if ( evenX == evenZ ) {
            return color1;
        } else {
            return color2;
        }
    }

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const override {
        if ( diffuse < FLT_EPSILON ) {
            // if diffuse is 0, do reflection
            ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
            return true;
        } else if ( specular < FLT_EPSILON ) {
            // if specular is 0, do diffuse reflection
            ray_out = Ray( I, DiffuseReflection( N ) );
            return false;
        }

        // if both are > 0 we pick one randomly
        if ( RandomFloat() < specular ) {
            ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
            return true;
        }

        ray_out = Ray( I, DiffuseReflection( N ) );
        return false;
    }

    virtual float* getColorModifier( Ray& ray_in, float3 N ) const {
        float3 I = ray_in.IntersectionPoint();
        bool evenX = abs( ( (int) floor( I.x ) ) % 2 ) == 0;
        bool evenZ = abs( ( (int) floor( I.z ) ) % 2 ) == 0;
        if ( evenX == evenZ ) {
            float results[4] = { color1.x, color1.y, color1.z, diffuse };
            return new float[4] { color1.x, color1.y, color1.z, diffuse };
        } else {
            float results[4] = { color2.x, color2.y, color2.z, diffuse };
            return new float[4] { color2.x, color2.y, color2.z, diffuse };
        }
    }

private:
    float3 color1;
    float3 color2;
    float diffuse;
    float specular;
};