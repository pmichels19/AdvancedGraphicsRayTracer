#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init() {
    // create fp32 rgb pixel buffer to render to
    accumulator = (float4*) MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
    memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );

    printf( PACKET_TRAVERSAL ? "Tracing using ray packets. Whitted style not supported for this.\n" : "Switch between Whitted and Path tracing using the 'K' key.\n" );
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace( Ray& ray, bool lastSpecular, int depth ) {
    if ( depth == 0 ) return float3( 0 );
    // intersect the ray with the scene
    scene.IntersectBVH( ray );
    //scene.FindNearest( ray );

    // if we hit nothing return a sky color
    if ( ray.objIdx == -1 ) return skyColor( ray.D );

    // fetch intersection point, normal and material
    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    //return ( N + 1 ) * 0.5f;
    ObjectMaterial* mat = scene.GetMaterial( ray.objIdx );

    // from material we can get a bounce ray, an albedo, whether the bounce is from a specular surface and the type of material
    Ray ray_out;
    bool specularBounce = mat->scatter( ray, I, N, ray_out );
    float3 albedo = mat->GetColor( ray );
    MaterialType flag = mat->getFlag();

    if ( flag == DIFFUSE ) {
        // === DIFFUSE ===
        float3 BRDF = albedo * INVPI;
        float PDF = INV2PI;
        float3 Ld = NextEventDirectIllumination( I, N, BRDF );
        float3 Ei = Trace( ray_out, specularBounce, depth - 1 ) * dot( N, ray_out.D ) / PDF;
        return BRDF * Ei + Ld;
    } else if ( flag == SPECULAR ) {
        // === SPECULAR ===
        return albedo * Trace( ray_out, specularBounce, depth - 1 );
    } else if ( flag == MIX ) {
        if ( specularBounce ) {
            // === SPECULAR ===
            return albedo * Trace( ray_out, specularBounce, depth - 1 );
        }

        // === DIFFUSE ===
        float3 BRDF = albedo * INVPI;
        float PDF = INV2PI;
        float3 Ld = NextEventDirectIllumination( I, N, BRDF );
        float3 Ei = Trace( ray_out, specularBounce, depth - 1 ) * dot( N, ray_out.D ) / PDF;
        return BRDF * Ei + Ld;
    } else if ( flag == DIELECTRIC ) {
        // === DIELECTRIC ===
        return albedo * Trace( ray_out, specularBounce, depth - 1 );
    } else if ( flag == LIGHT ) {
        // === LIGHT ===
        if ( lastSpecular ) return albedo;
        return float3( 0.0f );
    }

    // we shouldn't be able to get here
    printf("!!! path tracer ran into undefined material flag !!!\n");
    return float3( 0.0f );
}

vector<float3> Renderer::TracePacket( RayPacket& rays ) {
    // intersect the rays with the scene
    scene.IntersectBVHPacket( rays );

    vector<float3> result( PACKET_SIZE );
    for ( int rayIdx = 0; rayIdx < PACKET_SIZE; rayIdx++ ) {
        // if we hit nothing return a sky color
        if ( rays.objIdx[rayIdx] == -1 ) {
            result[rayIdx] = skyColor(rays.D[rayIdx]);
            continue;
        }

        // otherwise get prefilled ray from the packet
        Ray ray = rays.GetRay( rayIdx );
        // TODO: everything from here can go in a seperate function, save that for RR implementation :)
        // fetch intersection point, normal and material
        float3 I = ray.O + ray.t * ray.D;
        float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
        ObjectMaterial* mat = scene.GetMaterial( ray.objIdx );

        // from material we can get a bounce ray, an albedo, whether the bounce is from a specular surface and the type of material
        Ray ray_out;
        bool specularBounce = mat->scatter( ray, I, N, ray_out );
        float3 albedo = mat->GetColor( ray );
        MaterialType flag = mat->getFlag();

        if ( flag == DIFFUSE ) {
            // === DIFFUSE ===
            float3 BRDF = albedo * INVPI;
            float PDF = INV2PI;
            float3 Ld = NextEventDirectIllumination( I, N, BRDF );
            float3 Ei = Trace( ray_out, specularBounce ) * dot( N, ray_out.D ) / PDF;
            result[rayIdx] = BRDF * Ei + Ld;
        } else if ( flag == SPECULAR ) {
            // === SPECULAR ===
            result[rayIdx] = albedo * Trace( ray_out, specularBounce );
        } else if ( flag == MIX ) {
            if ( specularBounce ) {
                // === SPECULAR ===
                result[rayIdx] = albedo * Trace( ray_out, specularBounce );
            }

            // === DIFFUSE ===
            float3 BRDF = albedo * INVPI;
            float PDF = INV2PI;
            float3 Ld = NextEventDirectIllumination( I, N, BRDF );
            float3 Ei = Trace( ray_out, specularBounce ) * dot( N, ray_out.D ) / PDF;
            result[rayIdx] = BRDF * Ei + Ld;
        } else if ( flag == DIELECTRIC ) {
            // === DIELECTRIC ===
            result[rayIdx] = albedo * Trace( ray_out, specularBounce );
        } else if ( flag == LIGHT ) {
            // === LIGHT ===
            result[rayIdx] = albedo;
        } else {
            // we shouldn't be able to get here
            printf( "!!! packet tracer ran into undefined material flag !!!\n" );
            result[rayIdx] = float3( 0.0f );
        }
    }

    return result;
}

float3 Renderer::WhittedTrace( Ray& ray, int depth ) {
    float3 result = float3( 0 );
    if ( depth == 0 ) return result;

    scene.IntersectBVH( ray );
    if ( ray.objIdx == -1 ) return skyColor( ray.D );

    float3 I = ray.O + ray.t * ray.D;
    float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
    ObjectMaterial* mat = scene.GetMaterial( ray.objIdx );

    MaterialType flag = mat->getFlag();
    float* colorVars = mat->getColorModifier( ray, N );
    if ( flag == MaterialType::LIGHT ) {
        result += scene.GetLightColor();
    } else if ( flag == MaterialType::DIFFUSE ) {
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
        result += ( 1.0f - colorVars[3] ) * WhittedTrace( reflectionRay, depth - 1 );
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
    float totalEnergy = 0.0f;
    // lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
#if !PACKET_TRAVERSAL
    for ( int y = 0; y < SCRHEIGHT; y += 4 ) {
        // trace a primary ray for each pixel on the line
        for ( int x = 0; x < SCRWIDTH; x += 4 ) {
            for ( int v = 0; v < 4; v++ ) {
                for ( int u = 0; u < 4; u++ ) {
                    int samples;
                    float3 result = float3( 0 );
                    for ( samples = 0; samples < 1; samples++ ) { // iterate to 1 to disable anti-aliasing, going higher will drastically impact performance
                        if ( animation ) scene.SetTime( animTime + Rand( deltaTime * 0.002f ) );
                        if ( useWhitted ) {
                            result += WhittedTrace( camera.GetPrimaryRay( x + u, y + v ) );
                        } else {
                            result += Trace( camera.GetPrimaryRay( x + u, y + v ) );
                        }
                    }

                    int accIdx = ( x + u ) + ( y + v ) * SCRWIDTH;

                    if ( !stationary || animation ) accumulator[accIdx] = 0;
                    // increment the hit count so we don't divide by 0
                    accumulator[accIdx].w += 1;
                    // take the average over all hits
                    accumulator[accIdx] = accumulator[accIdx] + ( 1.0f / accumulator[accIdx].w ) * ( float4( ( 1.0f / (float) samples ) * result, accumulator[accIdx].w ) - accumulator[accIdx] );
                    // translate accumulator contents to rgb32 pixels
                    screen->pixels[accIdx] = RGBF32_to_RGB8( &accumulator[accIdx] );
                    totalEnergy += accumulator[accIdx].x + accumulator[accIdx].y + accumulator[accIdx].z;
                }
            }
        }
}
#else
    for ( int y = 0; y < SCRHEIGHT; y += SQRT_PACKET_SIZE ) {
        for ( int x = 0; x < SCRWIDTH; x += SQRT_PACKET_SIZE ) {
            // create packet
            RayPacket packet;
            for ( int yp = 0; yp < SQRT_PACKET_SIZE; yp++ ) {
                if ( y + yp >= SCRHEIGHT ) break;

                for ( int xp = 0; xp < SQRT_PACKET_SIZE; xp++ ) {
                    if ( x + xp >= SCRWIDTH ) continue;

                    int pIdx = xp + yp * SQRT_PACKET_SIZE;
                    // get a ray from the camera
                    Ray ray = camera.GetPrimaryRay( x + xp, y + yp );
                    packet.O[pIdx] = ray.O;
                    packet.D[pIdx] = ray.D;
                    packet.t[pIdx] = ray.t;
                }
            }

            vector<float3> pixels = TracePacket( packet );
            for ( int yp = 0; yp < SQRT_PACKET_SIZE; yp++ ) {
                if ( y + yp >= SCRHEIGHT ) break;

                for ( int xp = 0; xp < SQRT_PACKET_SIZE; xp++ ) {
                    if ( x + xp >= SCRWIDTH ) continue;

                    int pIdx = xp + yp * SQRT_PACKET_SIZE;
                    int accIdx = ( x + xp ) + ( y + yp ) * SCRWIDTH;

                    float3 color = pixels[pIdx];
                    //printf( "(%d, %d)\t\t(%f, %f, %f)\n", x + xp, y + yp, color.x, color.y, color.z );
                    if ( !stationary || animation ) accumulator[accIdx] = 0;
                    // increment the hit count so we don't divide by 0
                    accumulator[accIdx].w += 1;
                    // take the average over all hits
                    accumulator[accIdx] = accumulator[accIdx] + ( 1.0f / accumulator[accIdx].w ) * ( float4( color, accumulator[accIdx].w ) - accumulator[accIdx] );
                    // translate accumulator contents to rgb32 pixels
                    screen->pixels[accIdx] = RGBF32_to_RGB8( &accumulator[accIdx] );
                    totalEnergy += accumulator[accIdx].x + accumulator[accIdx].y + accumulator[accIdx].z;
                }
            }
        }
    }
#endif
    if (animation) scene.SetTime( animTime += deltaTime * 0.002f );
    if ( tracerSwap ) tracerSwap = !tracerSwap; // if we tossed away the accumulator this frame we want to make sure to start it again next frame with the new tracer
    // performance report - running average - ms, MRays/s
    static float avg = 10, alpha = 1;
    avg = ( 1 - alpha ) * avg + alpha * t.elapsed() * 1000;
    if ( alpha > 0.05f ) alpha *= 0.5f;
    float fps = 1000 / avg, rps = ( SCRWIDTH * SCRHEIGHT ) * fps;
    printf( "%5.2fms (%.1fps) - %.1fMrays/s\t\t%.1f\n", avg, fps, rps / 1000000, totalEnergy );
}