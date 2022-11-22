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
    float3 result = float3( 0, 0, 0 );
    if ( depth == 0 ) {
        return result;
    }

    scene.FindNearest( ray );
    if ( ray.objIdx == -1 ) return 0; // or a fancy sky color
    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    float3 albedo = scene.GetAlbedo( ray.objIdx, I );
    Material mat = scene.GetMaterial( ray.objIdx );
    /* visualize normal */ // return ( N + 1 ) * 0.5f;
    /* visualize distance */ // return 0.1f * float3( ray.t, ray.t, ray.t );
    /* visualize albedo */ // return albedo;
    if ( mat.specular > FLT_EPSILON ) {
        Ray reflectionRay = Ray( I, normalize( ray.D - 2 * dot( ray.D, N ) * N ) );
        result = result + mat.specular * Trace( reflectionRay, depth - 1 );
    }

    if ( mat.diffuse > FLT_EPSILON ) {
        result = result + mat.diffuse * DirectIllumination( I, N );
    }

    return mat.color * result;
}

float3 Renderer::DirectIllumination( float3 I, float3 N ) {
    float3 intersectionToLight = scene.GetLightPos() - I;
    float distance = length( intersectionToLight );
    Ray toLight = Ray( I, normalize( intersectionToLight ), distance );

    if ( scene.IsOccluded( toLight ) ) {
        return float3( 0, 0, 0 );
    }

    return ( dot( toLight.D, N ) / (distance * distance) ) * scene.GetLightColor();
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime ) {
    // animation
    static float animTime = 0;
    scene.SetTime( animTime += deltaTime * 0.002f );
    // move the camera based on inputs given
    camera.AdjustCamera( yaw, pitch, roll, xMove, yMove, zMove );
    // pixel loop
    Timer t;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
    for ( int y = 0; y < SCRHEIGHT; y++ ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x++ ) {
            accumulator[x + y * SCRWIDTH] = float4( Trace( camera.GetPrimaryRay( x, y ) ), 0 );
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
    // printf( "%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000 );
}