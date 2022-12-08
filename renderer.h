#pragma once

namespace Tmpl8 {

    class Renderer: public TheApp {
    public:
        // game flow methods
        void Init();
        float3 Trace( Ray& ray, int depth = 50 );

        float3 skyColor( float3 D ) {
            uint u = skydome->width * atan2f( D.z, D.x ) * INV2PI - 0.5f;
            uint v = skydome->height * acosf( D.y ) * INVPI - 0.5f;
            uint skyIdx = ( u & ( skydome->width - 1 ) ) + ( v & ( skydome->height - 1 ) ) * skydome->width;
            uint p = skydome->pixels[skyIdx];
            uint3 i3( ( p >> 16 ) & 255, ( p >> 8 ) & 255, p & 255 );
            return float3( i3 ) * SKYDOME_CORREDCTION;
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
    };

} // namespace Tmpl8