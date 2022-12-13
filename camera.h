#pragma once

// default screen resolution
#define SCRWIDTH	1280
#define SCRHEIGHT	720
// #define FULLSCREEN
// #define DOUBLESIZE

namespace Tmpl8 {

    class Camera {
    private:
        void applyMatrix(mat4 matrix) {
            camPos      = matrix.TransformPoint( camPos );
            topLeft     = matrix.TransformPoint( topLeft );
            topRight    = matrix.TransformPoint( topRight );
            bottomLeft  = matrix.TransformPoint( bottomLeft );
        }

        float3 randomInUnitDisk() {
            while ( true ) {
                float3 p = float3( RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, 0 );
                if ( sqrLength( p ) >= 1 ) continue;
                return p;
            }
        }
    public:
        Camera() {
            aperture = 0.05;
            lensRadius = aperture / 2.0f;
            focusDistance = 1;

            FOV = 1;

            // setup a basic view frustum
            camPos = float3( 0, 0, -FOV );
            topLeft = camPos + focusDistance * float3( -aspect, 1, FOV );
            topRight = camPos + focusDistance * float3( aspect, 1, FOV );
            bottomLeft = camPos + focusDistance * float3( -aspect, -1, FOV );
            totalRotation = mat4::Identity();
        }

        Ray GetPrimaryRay( const int x, const int y ) {
            // calculate pixel position on virtual screen plane
            const float u = (float) x * rWidth + Rand( rWidth );
            const float v = (float) y * rHeight + Rand( rHeight );

            float3 rd = lensRadius * randomInUnitDisk();
            const float3 offset = float3( u * rd.x, v * rd.y, 0 );
            const float3 P = topLeft + u * ( topRight - topLeft ) + v * ( bottomLeft - topLeft );
            return Ray( camPos + offset, normalize( P - camPos - offset ) );
        }

        void AdjustCamera( float yaw, float pitch, float roll, float xMove, float yMove, float zMove ) {
            // 1. undo translations so far done to camera
            mat4 currTranslation = mat4::Translate( camPos );
            mat4 undoTranslation = mat4::Translate( -camPos );
            applyMatrix(undoTranslation);

            // 2. undo rotations so far done to camera
            mat4 undoRotation = totalRotation.Inverted();
            applyMatrix(undoRotation);

            // 3. build camera matrix and apply based on inputs
            // new rotation(s)
            mat4 rotation = mat4::Identity();
            if ( abs( pitch ) > FLT_EPSILON ) rotation = rotation * mat4::RotateX( -pitch * 0.1f );
            if ( abs( yaw ) > FLT_EPSILON ) rotation = rotation * mat4::RotateY( -yaw * 0.1f );
            if ( abs( roll ) > FLT_EPSILON ) rotation = rotation * mat4::RotateZ( -roll * 0.1f );
            // and translation(s)
            mat4 cameraMatrix = mat4(rotation);
            cameraMatrix( 0, 3 ) = 0.1f * xMove;
            cameraMatrix( 1, 3 ) = 0.1f * yMove;
            cameraMatrix( 2, 3 ) = 0.1f * zMove;
            // apply
            applyMatrix(cameraMatrix);

            // 4. reset rotations already applied
            applyMatrix(totalRotation);

            // 5. reset translations already applied
            applyMatrix(currTranslation);

            // 6. update totalRotation
            totalRotation = totalRotation * rotation;
        }

        // DOF stuff
        float aperture;
        float focusDistance;
        float lensRadius;
        float FOV;

        float aspect = (float) SCRWIDTH / (float) SCRHEIGHT;
        float rWidth = 1.0f / SCRWIDTH;
        float rHeight = 1.0f / SCRHEIGHT;
        float3 camPos;
        float3 topLeft, topRight, bottomLeft;
        mat4 totalRotation;
    };

}