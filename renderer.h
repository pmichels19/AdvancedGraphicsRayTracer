#pragma once

namespace Tmpl8 {

    class Renderer: public TheApp {
    public:
        // game flow methods
        void Init();
        float3 Trace( Ray& ray, bool lastSpecular = true, int depth = 10 );

        vector<float3> TracePacket( RayPacket& rays );

        float3 WhittedTrace( Ray& ray, int depth = 20 );

        float3 DepthTrace( Ray& ray ) {
            int steps = scene.BVHDepth( ray );
            //printf("%d\n", steps );
            return float3( steps, 1.0f, 0.0f );
        }

        float3 skyColor( float3 D ) {
            uint u = skydome->width * atan2f( D.z, D.x ) * INV2PI - 0.5f;
            uint v = skydome->height * acosf( D.y ) * INVPI - 0.5f;
            uint skyIdx = ( u & ( skydome->width - 1 ) ) + ( v & ( skydome->height - 1 ) ) * skydome->width;
            uint p = skydome->pixels[skyIdx];
            uint3 i3( ( p >> 16 ) & 255, ( p >> 8 ) & 255, p & 255 );
            return float3( i3 ) * SKYDOME_CORRECTION;
        }

        float3 DirectIllumination( float3 I, float3 N ) {
            float3 result = float3( 0 );
            int samples = 0;
            for ( samples; samples < 4; samples++ ) {
                float3 intersectionToLight = scene.GetLightPos( 0 ) - I;
                float distance = length( intersectionToLight );
                intersectionToLight /= distance;
                float dotDN = dot( intersectionToLight, N );

                if ( dotDN < 0 || dot( scene.GetLightDir(), intersectionToLight ) > 0 ) continue;

                Ray toLight = Ray( I, intersectionToLight, distance - ( 2 * EPS ) );
                if ( scene.IsOccluded( toLight ) ) continue;

                result += ( dotDN / ( distance * distance ) ) * scene.GetLightColor();
            }

            return result / (float) samples;
        }

        float3 NextEventDirectIllumination( float3 I, float3 N, float3 BRDF ) {
            // pick a random light
            int light = scene.GetRandomLight();
            // then a random position on that light
            float3 Ilight = scene.GetLightPos( light );
            // get the area
            float area = scene.GetArea( light );
            // get the vector from I to the random postion on the light
            float3 L = Ilight - I;
            // get the distance from I to that random postion
            float distance = length( L );
            // normalize
            L /= distance;
            // get the normal at the randomly chosen point
            float3 Nl = scene.GetNormal( light, Ilight, L );

            float dotNL = dot( N, L );
            float dotNlL = dot( Nl, -L );
            float3 Ld = float3( 0.0f );
            if ( dotNL > 0 && dotNlL > 0 ) {
                // make a ray from I to the light source
                Ray toLight( I, L, distance - 2.0f * EPS );

                if ( !scene.IsOccluded( toLight ) ) {
                    float solidAngle = ( dotNlL * area ) / ( distance * distance );
                    float lightPDF = 1.0f / solidAngle;
                    Ld = scene.GetLightColor( light, toLight ) * BRDF * ( dotNL / lightPDF ); // TODO: currently we have only one hardcoded light...that might change
                }
            }

            return Ld;
        }

        void Tick( float deltaTime );

        void Shutdown() { /* implement if you want to do something on exit */ }

        // input handling
        void MouseUp( int button ) {
            /* implement if you want to detect mouse button presses */ 
        }
        void MouseDown( int button ) { 
            /* implement if you want to detect mouse button presses */
        }
        void MouseMove( int x, int y ) {
            mousePos.x = x, mousePos.y = y;
        }
        void MouseWheel( float y ) {
            /* implement if you want to handle the mouse wheel */
        }
        void KeyUp( int key ) {
            // yaw
            if ( key == 262 ) yaw += 1; // right arrow
            if ( key == 263 ) yaw -= 1; // left arrow
            // pitch
            if ( key == 265 ) pitch -= 1; // up arrow
            if ( key == 264 ) pitch += 1; // down arrow
            // roll
            if ( key == 81 ) roll += 1; // q
            if ( key == 69 ) roll -= 1; // e

            // xMove
            if ( key == 65 ) xMove += 1; // a
            if ( key == 68 ) xMove -= 1; // d
            // yMove
            if ( key == 32 ) yMove -= 1; // space
            if ( key == 341 ) yMove += 1; // ctrl
            // zMove
            if ( key == 87 ) zMove -= 1; // w
            if ( key == 83 ) zMove += 1; // s
        }
        void KeyDown( int key ) {
            // yaw
            if ( key == 262 ) yaw -= 1; // right arrow
            if ( key == 263 ) yaw += 1; // left arrow
            // pitch
            if ( key == 265 ) pitch += 1; // up arrow
            if ( key == 264 ) pitch -= 1; // down arrow
            // roll
            if ( key == 81 ) roll -= 1; // q
            if ( key == 69 ) roll += 1; // e

            // xMove
            if ( key == 65 ) xMove -= 1; // a
            if ( key == 68 ) xMove += 1; // d
            // yMove
            if ( key == 32 ) yMove += 1; // space
            if ( key == 341 ) yMove -= 1; // ctrl
            // zMove
            if ( key == 87 ) zMove += 1; // w
            if ( key == 83 ) zMove -= 1; // s

#if !PACKET_TRAVERSAL
            // switching ray tracer styles with K
            if ( key == 75 ) useWhitted = !useWhitted, tracerSwap = true, printf( "Switching to %s.\n", useWhitted ? "Whitted style ray tracer" : "Kajiya path tracer" );
#endif
        }

        // data members
        int2 mousePos;
        float4* accumulator;
        Scene scene;
        Camera camera;

        float yaw   = 0;
        float pitch = 0;
        float roll  = 0;

        float xMove = 0;
        float yMove = 0;
        float zMove = 0;

        int stationaryFrames = 0;

        bool useWhitted = false;
        bool tracerSwap = false;
    };

} // namespace Tmpl8