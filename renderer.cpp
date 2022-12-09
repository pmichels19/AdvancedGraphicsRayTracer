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
        //scene.FindNearest( ray );
        scene.IntersectBVH( ray, 0 );

        // if we hit nothing return a sky color
        if ( ray.objIdx == -1 ) {
            return result * skyColor( ray.D );
        }

        // fetch intersection point, normal and material
        float3 I = ray.O + ray.t * ray.D;
        float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
        shared_ptr<ObjectMaterial> mat = scene.GetMaterial( ray.objIdx );

        float3 color;
        Ray ray_out;
        continues = mat->bounce( ray, I, N, color, ray_out );
        result *= color;
        if ( !continues ) return result;

        ray = ray_out;
    }

    return result;
}

float3 Renderer::WhittedTrace( Ray& ray, int depth ) {
    float3 result = float3( 0 );
    if ( depth == 0 ) return result;

    scene.FindNearest( ray );
    if ( ray.objIdx == -1 ) return skyColor( ray.D );

    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    shared_ptr<ObjectMaterial> mat = scene.GetMaterial( ray.objIdx );

    MaterialType flag = mat->getFlag();
    float* colorVars = mat->getColorModifier( ray, N );
    if ( flag == MaterialType::DIFFUSE ) {
        // purely diffuse
        result += DirectIllumination( I, N );
    } else if ( flag == MaterialType::SPECULAR ) {
        // purely specular
        Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
        result += WhittedTrace( reflectionRay, depth - 1 );
    } else if ( flag == MaterialType::MIX ) {
        // diffuse bit
        result += colorVars[3] * DirectIllumination( I, N );
        // reflection bit
        Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
        result += ( 1.0f - colorVars[3] ) * WhittedTrace(reflectionRay, depth - 1);
    } else if ( flag == MaterialType::DIELECTRIC ) {
        if ( colorVars[3] < 0 ) {
            // handle Total Internal Reflection (TIR), indicated by a colorModifier with values of -1
            Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
            reflectionRay.inside = true;
            result += WhittedTrace( reflectionRay, depth - 1 );
        } else {
            float Fr = colorVars[3];
            float Ft = 1 - Fr;

            // reflection
            if ( Fr > FLT_EPSILON ) {
                float3 R = normalize( reflect( ray.D, N ) );
                Ray reflectionRay = Ray( I, R );
                result += Fr * WhittedTrace( reflectionRay, depth - 1 );
            }

            // refraction
            if ( Ft > FLT_EPSILON ) {
                Ray refractionRay = Ray( I, float3( colorVars[4], colorVars[5], colorVars[6] ) );
                refractionRay.inside = !ray.inside;
                result += Ft * WhittedTrace( refractionRay, depth - 1 );
            }
        }
    }

    float3 color = float3( colorVars[0], colorVars[1], colorVars[2] );
    delete colorVars;
    return color * result;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime ) {
    // animation
    static float animTime = 0;
    // move the camera based on inputs given
    bool stationary = abs(yaw) < FLT_EPSILON && abs(pitch) < FLT_EPSILON && abs(roll) < FLT_EPSILON 
        && abs(xMove) < FLT_EPSILON && abs(yMove) < FLT_EPSILON && abs(zMove) < FLT_EPSILON && !tracerSwap;
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
                if ( useWhitted ) {
                    result += WhittedTrace( camera.GetPrimaryRay( x, y ) );
                } else {
                    result += Trace( camera.GetPrimaryRay( x, y ) );
                }
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
    if ( tracerSwap ) tracerSwap = !tracerSwap; // if we tossed away the accumulator this frame we want to make sure to start it again next frame with the new tracer
    // performance report - running average - ms, MRays/s
    static float avg = 10, alpha = 1;
    avg = ( 1 - alpha ) * avg + alpha * t.elapsed() * 1000;
    if ( alpha > 0.05f ) alpha *= 0.5f;
    float fps = 1000 / avg, rps = ( SCRWIDTH * SCRHEIGHT ) * fps;
    printf( "%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000 );
}