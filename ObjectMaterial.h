#pragma once

class ObjectMaterial {
public:
    virtual MaterialType getFlag() const = 0;

    virtual float3 GetColor( Ray& ray_in ) const = 0;

    virtual bool scatter( Ray& ray_in, float3 I, float3 N, Ray& ray_out ) const = 0;

    // WHITTED STYLE RAY TRACER SUPPORT
    virtual float* getColorModifier( Ray& ray_in, float3 N ) const = 0;

    // ===================================================
    // helper methods we might need in material subclasses
    // ===================================================

    float3 DiffuseReflection( float3 N ) const {
        // normally distributed ray direction within the hemisphere
        while ( true ) {
            float3 R = float3( RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f );
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