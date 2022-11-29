#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init() {
    // create fp32 rgb pixel buffer to render to
    accumulator = (float4*) MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
    memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );

    previousMovingHits = ( int* ) MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
    memset( previousMovingHits, 0, SCRWIDTH * SCRHEIGHT * 16 );
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace( Ray& ray, int depth ) {
    float3 result = float3( 0 );
    if ( depth == 0 ) {
        return 0;
    }

    scene.FindNearest( ray );
    if ( ray.objIdx == -1 ) return 0; // or a fancy sky color
    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    // float3 albedo = scene.GetAlbedo( ray.objIdx, I );
    Material* mat = scene.GetMaterial( ray.objIdx );
    float3 matColor = mat->GetColor(I);
    /* visualize normal */ // return ( N + 1 ) * 0.5f;
    /* visualize distance */ // return 0.1f * float3( ray.t, ray.t, ray.t );
    /* visualize albedo */ // return albedo;
    if ( mat->n < FLT_EPSILON ) {
        // non-dielectric material, just do normal diffuse and specularity
        if ( mat->specular > FLT_EPSILON ) {
            Ray reflectionRay = Ray( I, normalize( reflect( ray.D, N ) ) );
            result += mat->specular * Trace( reflectionRay, depth - 1 );
        }

        if ( mat->diffuse > FLT_EPSILON ) {
            result += mat->diffuse * DirectIllumination( I, N );
        }
    } else {
        float n1 = 1;
        float n2 = mat->n;
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
            float Fr = 0;
            if ( !ray.inside ) {
                float sini = length( cross( N, ray.D ) );
                float cost = sqrtf( 1 - sqrf( n1Divn2 * sini ) );

                float Fr = Fresnel( n1, n2, cost, -cosi );
            }

            float Ft = 1 - Fr;

            // reflection
            if ( Fr > FLT_EPSILON ) {
                float3 R = normalize( reflect( ray.D, N ) );
                Ray reflectionRay = Ray( I, R );
                result += Fr * Trace( reflectionRay, depth - 1 );
            }

            // refraction
            if ( Ft > FLT_EPSILON ) {
                float3 T = normalize( n1Divn2 * ray.D - ( n1Divn2 * cosi + sqrtf( k ) ) * N );
                Ray refractionRay = Ray( I, T );
                refractionRay.inside = !ray.inside; // TODO: only when hitting object with volume, maybe material property?
                result += Ft * Trace( refractionRay, depth - 1 );
            }
        }

        // Beer's law, use the color as absorbance instead of the actual color
        if ( ray.inside ) {
            result.x = result.x * exp( -matColor.x * ray.t );
            result.y = result.y * exp( -matColor.y * ray.t );
            result.z = result.z * exp( -matColor.z * ray.t );
        }

        return result;
    }

    //return mat.color * result;
    if ( ray.objIdx == 9 ) {
        return scene.GetAlbedo(9, I) * result;
    }
    return matColor * result;
}

float3 Renderer::DirectIllumination( float3 I, float3 N ) {
    float3 result = float3( 0 );
    int samples = 0;
    for (samples; samples < 4; samples++) {
        float3 intersectionToLight = scene.GetLightPos() - I;
        float distance = length(intersectionToLight);
        intersectionToLight /= distance;
        float dotDN = dot(intersectionToLight, N);

        if (dotDN < 0) {
            continue;
        }

        Ray toLight = Ray(I, intersectionToLight, distance - 2 * EPS);
        // return black if no light source connects or if we are facing the occluded side of an object
        if (scene.IsOccluded(toLight)) {
            continue;
        }

        result += (dotDN / (distance * distance)) * scene.GetLightColor();
    }

    return result / (float) samples;
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

    if (!stationary) camera.AdjustCamera(yaw, pitch, roll, xMove, yMove, zMove);
    // pixel loop
    Timer t;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
    for ( int y = 0; y < SCRHEIGHT; y++ ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x++ ) {
            Ray ray = camera.GetPrimaryRay( x, y );
            scene.SetTime( animTime + Rand( deltaTime * 0.002f ) );
            float3 result = Trace( ray );
            //result += Trace( camera.GetPrimaryRay( x, y ) );
            //result += Trace( camera.GetPrimaryRay( x, y ) );
            //result += Trace( camera.GetPrimaryRay( x, y ) );

            int accIdx = x + y * SCRWIDTH;

            if ( !stationary ) {
                accumulator[accIdx] = 0;
            } else if ( ray.objIdx == 0 || ray.objIdx == 1 || ray.objIdx == 3 ) {
                accumulator[accIdx] = 0;
                previousMovingHits[accIdx] = 1;
            } else if ( previousMovingHits[accIdx] == 1 ) {
                accumulator[accIdx] = 0;
                previousMovingHits[accIdx] = 0;
            }

            // increment the hit count so we don't divide by 0
            accumulator[accIdx].w += 1;
            // take the average over all hits
            accumulator[accIdx] = accumulator[accIdx] + ( 1.0f / accumulator[accIdx].w ) * ( float4( result, accumulator[accIdx].w ) - accumulator[accIdx] );
        }

        // translate accumulator contents to rgb32 pixels
        for ( int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++ ) {
            screen->pixels[dest + x] = RGBF32_to_RGB8( &accumulator[x + y * SCRWIDTH] );
        }
    }

    scene.SetTime( animTime += deltaTime * 0.002f );
    // performance report - running average - ms, MRays/s
    static float avg = 10, alpha = 1;
    avg = ( 1 - alpha ) * avg + alpha * t.elapsed() * 1000;
    if ( alpha > 0.05f ) alpha *= 0.5f;
    float fps = 1000 / avg, rps = ( SCRWIDTH * SCRHEIGHT ) * fps;
    printf( "%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000 );
}