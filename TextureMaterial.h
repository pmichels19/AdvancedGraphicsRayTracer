#pragma once

class TextureMaterial: public ObjectMaterial {
public:
    TextureMaterial() = default;
    TextureMaterial( char* fileName, float diffuse, int fileWidth, int fileHeight, float width, float height )
    : fileName( fileName ), fileWidth( fileWidth ), fileHeight( fileHeight ), width( width ), height( height ) {
        this->diffuse = clamp( diffuse, 0.0f, 1.0f );
        specular = 1.0f - this->diffuse;
    }

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        static Surface texture( fileName );
        int ix = (int) ( (   I.x ) * ( fileWidth  / width  ) );
        int iy = (int) ( ( - I.y ) * ( fileHeight / height ) );
        int idx = ( ix & ( fileWidth - 1 ) ) + ( iy & ( fileHeight - 1 ) ) * fileWidth;
        uint p = texture.pixels[idx];
        uint3 i3( ( p >> 16 ) & 255, ( p >> 8 ) & 255, p & 255 );

        attenuation = float3( i3 ) * ( 1.0f / 255.0f );
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
    char* fileName;
    float diffuse;
    float specular;
    int fileWidth;
    int fileHeight;
    float width;
    float height;
};