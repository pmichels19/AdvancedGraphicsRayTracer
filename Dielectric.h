#pragma once

class Dielectric: public ObjectMaterial {
public:
    Dielectric() = default;
    Dielectric( const float3& absorption, const float& n ): absorption( absorption ), n( n ) {}

    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const override {
        float n1 = 1;
        float n2 = n;
        float n1Divn2 = n1 / n2;
        float cosi = dot( N, ray_in.D );
        attenuation = float3( 1 );
        if ( ray_in.inside ) {
            attenuation.x = exp( -absorption.x * ray_in.t );
            attenuation.y = exp( -absorption.y * ray_in.t );
            attenuation.z = exp( -absorption.z * ray_in.t );
            n1Divn2 = 1 / n1Divn2;
        }

        float k = 1 - ( n1Divn2 * n1Divn2 ) * ( 1 - ( cosi * cosi ) );
        // no choice when we TIR
        if ( k < 0 ) {
            ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
            ray_out.inside = true;
        } else {
            float Fr = 0;
            if ( !ray_in.inside ) {
                float sini = length( cross( N, ray_in.D ) );
                float cost = sqrtf( 1 - sqrf( n1Divn2 * sini ) );

                float Fr = solveFresnel( n1, n2, cost, -cosi );
            }

            if ( Fr > FLT_EPSILON && random_float( 0, 1 ) < Fr ) {
                ray_out = Ray( I, normalize( reflect( ray_in.D, N ) ) );
            } else {
                float3 T = normalize( n1Divn2 * ray_in.D - ( n1Divn2 * cosi + sqrtf( k ) ) * N );
                ray_out = Ray( I, T );
                ray_out.inside = !ray_in.inside; // TODO: only when hitting object with volume, maybe material property?
            }
        }

        return true;
    }

private:
    float3 absorption;
    float n;
};