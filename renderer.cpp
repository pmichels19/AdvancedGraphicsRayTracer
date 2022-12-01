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
    if ( depth == 0 ) {
        return 0;
    }

    // intersect the ray with the scene
    scene.FindNearest( ray );
    // if we hit nothing return black
    if ( ray.objIdx == -1 ) {
        float t = 0.5f * ( ray.D.y + 1.0f );
        return ( 1.0f - t ) * float3( 1.0f, 1.0f, 1.0f ) + t * float3( 0.5f, 0.7f, 1.0f );
        //return float3( 0 );
    }
    // fetch intersection point, normal and material
    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    Material* mat = scene.GetMaterial( ray.objIdx );
    float3 matColor = mat->GetColor( I );

    // if we hit a lightsource return its color, atm only supports the quad
    if ( ray.objIdx == 0 ) {
        if ( dot( scene.GetLightDir(), ray.D ) > 0 ) return float3( 0 );
        return scene.GetLightColor();
    }
    if ( mat->n > 1 - FLT_EPSILON ) {
        float n1 = 1;
        float n2 = mat->n;
        float n1Divn2 = n1 / n2;
        float cosi = dot( N, ray.D );
        float3 beers = float3( 1 );
        if ( ray.inside ) {
            beers.x = exp( -matColor.x * ray.t );
            beers.y = exp( -matColor.y * ray.t );
            beers.z = exp( -matColor.z * ray.t );
            n1Divn2 = 1 / n1Divn2;
        }

        float k = 1 - ( n1Divn2 * n1Divn2 ) * ( 1 - ( cosi * cosi ) );
        // no choice when we TIR
        if ( k < 0 ) {
            float3 R = normalize( reflect( ray.D, N ) );
            Ray TIRRay = Ray( I, R );
            TIRRay.inside = true;
            return beers * Trace( TIRRay, depth - 1 );
        }

        // random number we will use to decide what ray we are going to sample next
        float choice = random_float( 0, 1 );
        float Fr = 0;
        if ( !ray.inside ) {
            float sini = length( cross( N, ray.D ) );
            float cost = sqrtf( 1 - sqrf( n1Divn2 * sini ) );

            float Fr = Fresnel( n1, n2, cost, -cosi );
        }

        if ( Fr > FLT_EPSILON && choice < Fr ) {
            float3 R = normalize( reflect( ray.D, N ) );
            Ray reflectionRay = Ray( I, R );
            return matColor * Trace( reflectionRay, depth - 1 );
        }

        float3 T = normalize( n1Divn2 * ray.D - ( n1Divn2 * cosi + sqrtf( k ) ) * N );
        Ray refractionRay = Ray( I, T );
        refractionRay.inside = !ray.inside; // TODO: only when hitting object with volume, maybe material property?
        return beers * Trace( refractionRay, depth - 1 );
    } else if ( mat->specular > FLT_EPSILON && random_float( 0, 1 ) < mat->specular ) {
        Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
        return matColor * Trace( reflectionRay, depth - 1 );
    }

    // if we got here we do diffuse stuff
    float3 brdf = matColor / PI;
    float3 R = DiffuseReflection( N );
    Ray diffuseRay = Ray( I, R );
    float3 ei = Trace( diffuseRay, depth - 1 ) * dot( N, R );
    return PI * 2.0f * brdf * ei;
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
    bool animation = false; // Set to true for motion blur

    if (!stationary) camera.AdjustCamera(yaw, pitch, roll, xMove, yMove, zMove);
    // pixel loop
    Timer t;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
    for ( int y = 0; y < SCRHEIGHT; y++ ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x++ ) {
            Ray ray;
            int samples;
            float3 result = float3( 0 );
            for ( samples = 0; samples < 1; samples++ ) { // iterate to 1 to disable anti-aliasing
                if ( animation ) scene.SetTime( animTime + Rand( deltaTime * 0.002f ) );
                ray = camera.GetPrimaryRay( x, y );
                result += Trace( ray );
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