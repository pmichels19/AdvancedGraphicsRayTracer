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
    bool continues;
    float3 result = float3( 1.0f );
    for ( int d = depth; d >= 0; d-- ) {
        scene.FindNearest( ray );

        // if we hit nothing return black
        if ( ray.objIdx == -1 ) {
            float t = 0.5f * ( ray.D.y + 1.0f );
            result *= ( 1.0f - t ) * float3( 1.0f, 1.0f, 1.0f ) + t * float3( 0.5f, 0.7f, 1.0f );
            //result *= float3( 0 );
            break;
        }

        // fetch intersection point, normal and material
        float3 I = ray.O + ray.t * ray.D;
        float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
        shared_ptr<ObjectMaterial> mat = scene.GetMaterial( ray.objIdx );

        float3 color;
        Ray ray_out;
        continues = mat->bounce( ray, I, N, color, ray_out );
        result *= color;
        if ( !continues ) break;

        ray = ray_out;
    }

    return result;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime ) {
    // animation
    static float animTime = 0;
    // move the camera based on inputs given
    bool stationary = abs(yaw) < FLT_EPSILON && abs(pitch) < FLT_EPSILON && abs(roll) < FLT_EPSILON 
        && abs(xMove) < FLT_EPSILON && abs(yMove) < FLT_EPSILON && abs(zMove) < FLT_EPSILON;
    bool animation = false; // Set to true for animation with motion blur

    if (!stationary) camera.AdjustCamera(yaw, pitch, roll, xMove, yMove, zMove);
    // pixel loop
    Timer t;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
    for ( int y = 0; y < SCRHEIGHT; y++ ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x++ ) {
            int samples;
            float3 result = float3( 0 );
            for ( samples = 0; samples < 1; samples++ ) { // iterate to 1 to disable anti-aliasing, going higher will drastically impact performance
                if ( animation ) scene.SetTime( animTime + Rand( deltaTime * 0.002f ) );
                result += Trace( camera.GetPrimaryRay( x, y ) );
            }

            int accIdx = x + y * SCRWIDTH;

            if ( !stationary || animation ) accumulator[accIdx] = 0;
            // increment the hit count so we don't divide by 0
            accumulator[accIdx].w += 1;
            // take the average over all hits
            accumulator[accIdx] = accumulator[accIdx] + ( 1.0f / accumulator[accIdx].w ) * ( float4( ( 1.0f / (float) samples ) * result, accumulator[accIdx].w ) - accumulator[accIdx] );
        }

        // translate accumulator contents to rgb32 pixels
        for ( int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++ ) {
            screen->pixels[dest + x] = RGBF32_to_RGB8( &accumulator[x + y * SCRWIDTH] );
        }
    }

    if (animation) scene.SetTime( animTime += deltaTime * 0.002f );
    // performance report - running average - ms, MRays/s
    static float avg = 10, alpha = 1;
    avg = ( 1 - alpha ) * avg + alpha * t.elapsed() * 1000;
    if ( alpha > 0.05f ) alpha *= 0.5f;
    float fps = 1000 / avg, rps = ( SCRWIDTH * SCRHEIGHT ) * fps;
    printf( "%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000 );
}