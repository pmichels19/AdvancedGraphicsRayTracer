#pragma once

class ObjectMaterial {
public:
    virtual bool bounce( const Ray& ray_in, const float3 I, const float3 N, float3& attenuation, Ray& ray_out ) const = 0;

    // ===================================================
    // helper methods we might need in material subclasses
    // ===================================================

    float3 DiffuseReflection( float3 N ) const {
        // normally distributed ray direction within the hemisphere
        while ( true ) {
            float3 R = float3( random_float( -1.0f, 1.0f ), random_float( -1.0f, 1.0f ), random_float( -1.0f, 1.0f ) );
            if ( sqrLength( R ) > 1 ) continue;
            if ( dot( R, N ) < 0 ) R = -R;
            return normalize( R );
        }
    }

    float Fresnel( float n1, float n2, float cost, float cosi ) const {
        float sPolarized = ( n1 * cosi - n2 * cost ) / ( n1 * cosi + n2 * cost );
        float pPolarized = ( n1 * cost - n2 * cosi ) / ( n1 * cost + n2 * cosi );

        return 0.5f * ( ( sPolarized * sPolarized ) + ( pPolarized * pPolarized ) );
    }
};