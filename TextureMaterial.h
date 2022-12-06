#pragma once

class TextureMaterial: public ObjectMaterial {
public:
    TextureMaterial() = default;
    TextureMaterial( const char* filename ) {
        texture = new Surface( filename );
        printf( "Registered texture %s\n", filename );
        diffuse = 1.0f;
        specular = 0.0f;
    }

    TextureMaterial( const char* filename, float diffuse ) {
        texture = new Surface( filename );
        printf( "Registered texture %s\n", filename );
        this->diffuse = clamp( diffuse, 0.0f, 1.0f );
        specular = 1.0f - this->diffuse;
    }

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        uint u = (int) ( texture->width * ray_in.u ) % texture->width;
        uint v = (int) ( texture->height * ray_in.v ) % texture->height;
        uint skyIdx = ( u & ( texture->width - 1 ) ) + ( v & ( texture->height - 1 ) ) * texture->width;
        uint p = texture->pixels[skyIdx];
        uint3 i3( ( p >> 16 ) & 255, ( p >> 8 ) & 255, p & 255 );
        attenuation = float3( i3 ) * correction;

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

    Surface* texture;
    float diffuse;
    float specular;
    const float correction = 1.0f / 255.0f;
};