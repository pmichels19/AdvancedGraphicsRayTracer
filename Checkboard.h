#pragma once

class Checkboard: public Material {
public:
    Checkboard() = default;
    Checkboard( float diffuse, float3 color1 = float3( 0 ), float3 color2 = float3( 1 ) ): Material(color1, diffuse) {
        secondColor = color2;
    }

    virtual float3 GetColor( float3 I ) {
        bool evenX = abs( ( (int) floor( I.x ) ) % 2 ) == 0;
        bool evenZ = abs( ( (int) floor( I.z ) ) % 2 ) == 0;
        if ( evenX == evenZ ) {
            return color;
        }

        return secondColor;
    }

    float3 secondColor;
};
