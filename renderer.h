#pragma once

namespace Tmpl8 {

    class Renderer: public TheApp {
    public:
        // game flow methods
        void Init();
        float3 Trace( Ray& ray, int depth = 20 );
        float3 DirectIllumination( float3 I, float3 N );
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
        }

        float Fresnel( float n1, float n2, float cost, float cosi ) {
            float sPolarized = ( n1 * cosi - n2 * cost ) / ( n1 * cosi + n2 * cost );
            float pPolarized = ( n1 * cost - n2 * cosi ) / ( n1 * cost + n2 * cosi );
            //printf("%f, %f\n", sPolarized, pPolarized);

            return 0.5 * ( ( sPolarized * sPolarized ) + ( pPolarized * pPolarized ) );
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
    };

} // namespace Tmpl8