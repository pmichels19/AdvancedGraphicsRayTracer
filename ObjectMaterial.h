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
        //while ( true ) {
        //    float3 R = float3( RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f );
        //    if ( sqrLength( R ) > 1 ) continue;
        //    if ( dot( R, N ) < 0 ) R = -R;
        //    return normalize( R );
        //}

        float r0 = RandomFloat();
        float r1 = RandomFloat();

        float r = sqrtf( r0 );
        float z = sqrtf( 1 - r0 );

        float theta = TWOPI * r1;
        float x = r * cosf( theta );
        float y = r * sinf( theta );
        float3 direction( x, y, z );
        direction = normalize( direction );

        // set up the local axis system for N
        // FIXME: this...doesn't work :c
        float3 axisSystem[3];
        axisSystem[0] = float3( 0.0f, -1.0f, 0.0f );
        axisSystem[1] = float3( -1.0f, 0.0f, 0.0f );
        axisSystem[2] = N;
        if ( N.z + 1.0f > FLT_EPSILON ) {
            const float a = 1.0f / ( 1.0f + N.z );
            const float b = -N.x * N.y * a;
            axisSystem[1] = float3( 1.0f - N.x * N.x * a, b, -N.x );
            axisSystem[0] = float3( b, 1.0f - N.y * N.y * a, -N.y );
        }

        return normalize( direction.x * axisSystem[0] + direction.y * axisSystem[1] + direction.z * axisSystem[2] );
    }

    float Fresnel( float n1, float n2, float cost, float cosi ) const {
        float sPolarized = ( n1 * cosi - n2 * cost ) / ( n1 * cosi + n2 * cost );
        float pPolarized = ( n1 * cost - n2 * cosi ) / ( n1 * cost + n2 * cosi );

        return 0.5f * ( ( sPolarized * sPolarized ) + ( pPolarized * pPolarized ) );
    }
};