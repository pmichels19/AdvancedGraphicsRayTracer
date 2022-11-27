#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init() {
    // create fp32 rgb pixel buffer to render to
    accumulator = (float4*) MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
    memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace( Ray& ray, int depth ) {
    float3 result = float3( 0 );
    if ( depth == 0 ) {
        return result;
    }

    scene.FindNearest( ray );
    if ( ray.objIdx == -1 ) return 0; // or a fancy sky color
    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    // float3 albedo = scene.GetAlbedo( ray.objIdx, I );
    Material mat = scene.GetMaterial( ray.objIdx );
    /* visualize normal */ // return ( N + 1 ) * 0.5f;
    /* visualize distance */ // return 0.1f * float3( ray.t, ray.t, ray.t );
    /* visualize albedo */ // return albedo;
    if ( mat.n < FLT_EPSILON ) {
        // non-dielectric material, just do normal diffuse and specularity
        if ( mat.specular > FLT_EPSILON ) {
            Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
            result += mat.specular * Trace( reflectionRay, depth - 1 );
        }

        if ( mat.diffuse > FLT_EPSILON ) {
            result += mat.diffuse * DirectIllumination( I, N );
        }
    } else {
        float n1 = 1;
        float n2 = mat.n;
        float n1Divn2 = n1 / n2;
        float cosi = dot( N, ray.D );
        if ( ray.inside ) n1Divn2 = 1 / n1Divn2;

        float k = 1 - ( n1Divn2 * n1Divn2 ) * ( 1 - ( cosi * cosi ) );

        if ( k < 0 ) {
            // handle Total Internal Reflection (TIR)
            float3 R = normalize( reflect( ray.D, N ) );
            Ray TIRRay = Ray(I, R);
            TIRRay.inside = true;
            result += Trace( TIRRay, depth - 1 );
        } else {
            if ( ray.inside ) {
                float3 T = normalize( n1Divn2 * ray.D - ( n1Divn2 + sqrtf( k ) ) * N );
                Ray refractionRay = Ray( I, T );
                refractionRay.inside = !ray.inside;
                result += Trace( refractionRay, depth - 1 );
            }

            float sini = length( cross( N, ray.D ) );
            float cost = sqrtf( 1 - sqrf( n1Divn2 * sini ) );

            float Fr = Fresnel( n1, n2, cost, -cosi );
            float Ft = 1 - Fr;

            // reflection
            if ( Fr > FLT_EPSILON ) {
                float3 R = normalize( reflect( ray.D, N ) );
                Ray reflectionRay = Ray( I, R );
                result += Fr * Trace( reflectionRay, depth - 1 );
            }

            // refraction
            if ( Ft > FLT_EPSILON ) {
                float3 T = normalize( n1Divn2 * ray.D - ( n1Divn2 + sqrtf( k ) ) * N );
                Ray refractionRay = Ray( I, T );
                refractionRay.inside = !ray.inside;
                result += Ft * Trace( refractionRay, depth - 1 );
            }
        }

        // Beer's law, use the color as absorbance instead of the actual color
        if ( ray.inside ) {
            result.x = result.x * exp( -mat.color.x * ray.t );
            result.y = result.y * exp( -mat.color.y * ray.t );
            result.z = result.z * exp( -mat.color.z * ray.t );
        }
    }

    //return mat.color * result;
    return mat.color * result;
}

float3 Renderer::DirectIllumination( float3 I, float3 N ) {
    float3 intersectionToLight = scene.GetLightPos() - I;
    float distance = length( intersectionToLight );
    intersectionToLight = normalize( intersectionToLight );
    float dotDN = dot( intersectionToLight, N );

    if ( dotDN < 0 ) return float3( 0 );

    Ray toLight = Ray( I, intersectionToLight, distance );

    // return black if no light source connects or if we are facing the occluded side of an object
    if ( scene.IsOccluded( toLight ) ) {
        return float3( 0, 0, 0 );
    }

    return ( dotDN / (distance * distance) ) * scene.GetLightColor();
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime ) {
    // animation
    static float animTime = 0;
    //scene.SetTime( animTime += deltaTime * 0.002f );
    // move the camera based on inputs given
    camera.AdjustCamera( yaw, pitch, roll, xMove, yMove, zMove );
    // pixel loop
    Timer t;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
    for ( int y = 0; y < SCRHEIGHT; y++ ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x++ ) {
            float3 result = Trace( camera.GetPrimaryRay( x, y ) );
            //result += Trace( camera.GetPrimaryRay( x, y ) );
            //result += Trace( camera.GetPrimaryRay( x, y ) );
            //result += Trace( camera.GetPrimaryRay( x, y ) );

            accumulator[x + y * SCRWIDTH] = float4( result, 0 );
        }

        // translate accumulator contents to rgb32 pixels
        for ( int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++ ) {
            screen->pixels[dest + x] = RGBF32_to_RGB8( &accumulator[x + y * SCRWIDTH] );
        }
    }
    // performance report - running average - ms, MRays/s
    static float avg = 10, alpha = 1;
    avg = ( 1 - alpha ) * avg + alpha * t.elapsed() * 1000;
    if ( alpha > 0.05f ) alpha *= 0.5f;
    float fps = 1000 / avg, rps = ( SCRWIDTH * SCRHEIGHT ) * fps;
    printf( "%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000 );
}