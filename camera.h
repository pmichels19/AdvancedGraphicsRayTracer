#pragma once

// default screen resolution
#define SCRWIDTH	1280
#define SCRHEIGHT	720
// #define FULLSCREEN
// #define DOUBLESIZE

namespace Tmpl8 {

    class Camera {
    public:
        Camera() {
            // setup a basic view frustum
            camPos = float3( 0, 0, -2 );
            topLeft = float3( -aspect, 1, 0 );
            topRight = float3( aspect, 1, 0 );
            bottomLeft = float3( -aspect, -1, 0 );
            totalRotation = mat4::Identity();
        }

        Ray GetPrimaryRay( const int x, const int y ) {
            // calculate pixel position on virtual screen plane
            const float u = (float) x * ( 1.0f / SCRWIDTH );
            const float v = (float) y * ( 1.0f / SCRHEIGHT );
            const float3 P = topLeft + u * ( topRight - topLeft ) + v * ( bottomLeft - topLeft );
            return Ray( camPos, normalize( P - camPos ) );
        }

        void AdjustCamera( float yaw, float pitch, float roll, float xMove, float yMove, float zMove ) {
            mat4 rotation = mat4::Rotate( pitch, yaw, roll, 0.1 );
            totalRotation = totalRotation * rotation;
            float3 translation = totalRotation.TransformVector( 0.1 * float3( xMove, yMove, zMove ) );
            mat4 cameraMatrix = rotation * mat4::Translate( translation );

            mat4 camToWorld = mat4::Translate( -camPos);
            mat4 worldToCam = mat4::Translate(camPos);

            camPos = worldToCam.TransformPoint( cameraMatrix.TransformPoint( camToWorld.TransformPoint( camPos ) ) );
            printf("(%f, %f, %f)\n", camPos.x, camPos.y, camPos.z);
            topLeft = worldToCam.TransformPoint( cameraMatrix.TransformPoint( camToWorld.TransformPoint( topLeft ) ) );
            topRight = worldToCam.TransformPoint( cameraMatrix.TransformPoint( camToWorld.TransformPoint( topRight ) ) );
            bottomLeft = worldToCam.TransformPoint( cameraMatrix.TransformPoint( camToWorld.TransformPoint( bottomLeft ) ) );
        }

        float aspect = (float) SCRWIDTH / (float) SCRHEIGHT;
        float3 camPos;
        float3 topLeft, topRight, bottomLeft;
        mat4 totalRotation;
    };

}