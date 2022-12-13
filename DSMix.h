#pragma once

class DSMix: public ObjectMaterial {
public:
    DSMix() = default;
    DSMix( const float3& color, const float& diffuse ): color( color ) {
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
        return color;
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
        return new float[4] { color.x, color.y, color.z, diffuse };
    }

private:
    float3 color;
    float diffuse;
    float specular;
};