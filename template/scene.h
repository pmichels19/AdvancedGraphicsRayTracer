#pragma once

// -----------------------------------------------------------
// scene.h
// Simple test scene for ray tracing experiments. Goals:
// - Super-fast scene intersection
// - Easy interface: scene.FindNearest / IsOccluded
// - With normals and albedo: GetNormal / GetAlbedo
// - Area light source (animated), for light transport
// - Primitives can be hit from inside - for dielectrics
// - Can be extended with other primitives and/or a BVH
// - Optionally animated - for temporal experiments
// - Not everything is axis aligned - for cache experiments
// - Can be evaluated at arbitrary time - for motion blur
// - Has some high-frequency details - for filtering
// Some speed tricks that severely affect maintainability
// are enclosed in #ifdef SPEEDTRIX / #endif. Mind these
// if you plan to alter the scene in any way.
// -----------------------------------------------------------

#define SPEEDTRIX

#define PLANE_X(o,i) {if((t=-(ray.O.x+o)*ray.rD.x)<ray.t)ray.t=t,ray.objIdx=i;}
#define PLANE_Y(o,i) {if((t=-(ray.O.y+o)*ray.rD.y)<ray.t)ray.t=t,ray.objIdx=i;}
#define PLANE_Z(o,i) {if((t=-(ray.O.z+o)*ray.rD.z)<ray.t)ray.t=t,ray.objIdx=i;}

namespace Tmpl8 {

    __declspec( align( 64 ) )
        // -----------------------------------------------------------
        // Sphere primitive
        // Basic sphere, with explicit support for rays that start
        // inside it. Good candidate for a dielectric material.
        // -----------------------------------------------------------
        class Sphere
    {
    public:
        Sphere() = default;
        Sphere( int idx, float3 p, float r ):
            pos( p ), r2( r* r ), invr( 1 / r ), objIdx( idx ) {}
        void Intersect( Ray& ray ) const
        {
            float3 oc = ray.O - this->pos;
            float b = dot( oc, ray.D );
            float c = dot( oc, oc ) - this->r2;
            float t, d = b * b - c;
            if ( d <= 0 ) return;
            d = sqrtf( d ), t = -b - d;
            if ( t < ray.t && t > EPS )
            {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 cToI = normalize( ray.IntersectionPoint() - pos );
                ray.u = 0.5f - atan2f( cToI.z, cToI.x ) * INV2PI;
                ray.v = 0.5f - asinf( cToI.y ) * INVPI;
                return;
            }
            t = d - b;
            if ( t < ray.t && t > EPS )
            {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 cToI = normalize( ray.IntersectionPoint() - pos );
                ray.u = 0.5f - atan2f( cToI.z, cToI.x ) * INV2PI;
                ray.v = 0.5f - asinf( cToI.y ) * INVPI;
                return;
            }
        }
        bool Hit( Ray& ray ) {
            float3 oc = ray.O - this->pos;
            float b = dot( oc, ray.D );
            float c = dot( oc, oc ) - this->r2;
            float t, d = b * b - c;
            if ( d <= 0 ) return false;
            d = sqrtf( d ), t = -b - d;
            if ( t < ray.t && t > EPS ) return true;

            t = d - b;
            if ( t < ray.t && t > EPS ) return true;
            return false;
        }
        float3 GetNormal( const float3 I ) const
        {
            return ( I - this->pos ) * invr;
        }
        float3 GetAlbedo( const float3 I ) const
        {
            return float3( 0.93f );
        }

        // BVH building stuff
        float3 GetCentroid() const {
            return pos;
        }

        float3 GetAABBMin() const {
            return pos - float3( r2 * invr );
        }

        float3 GetAABBMax() const {
            return pos + float3( r2 * invr );
        }

        float GetArea() const {
            return 4.0f * PI * r2;
        }

        float3 GetRandomPoint() const {
            float3 point = float3( 1 );
            // normally distributed vector within a hemisphere
            while ( sqrLength( point ) > 1 ) {
                point = float3( RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f );
            }

            // scale it and return
            return normalize( point ) * r2 * invr;
        }

        float3 pos = 0;
        float r2 = 0, invr = 0;
        int objIdx = -1;
    };

    // -----------------------------------------------------------
    // Plane primitive
    // Basic infinite plane, defined by a normal and a distance
    // from the origin (in the direction of the normal).
    // -----------------------------------------------------------
    class Plane
    {
    public:
        Plane() = default;
        Plane( int idx, float3 normal, float dist ): N( normal ), d( dist ), objIdx( idx ) {}
        void Intersect( Ray& ray ) const
        {
            float t = -( dot( ray.O, this->N ) + this->d ) / ( dot( ray.D, this->N ) );
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 I = ray.IntersectionPoint();
                if ( N.x < FLT_EPSILON && N.y < FLT_EPSILON ) {
                    ray.u = I.x;
                    ray.v = -I.y;
                } else if ( N.x < FLT_EPSILON && N.z < FLT_EPSILON ) {
                    ray.u = I.x;
                    ray.v = -I.z;
                } else if ( N.y < FLT_EPSILON && N.z < FLT_EPSILON ) {
                    ray.u = I.y;
                    ray.v = -I.z;
                }
            }
        }
        bool Hit( Ray& ray ) {
            float t = -( dot( ray.O, this->N ) + this->d ) / ( dot( ray.D, this->N ) );
            return t < ray.t&& t > EPS;
        }
        float3 GetNormal( const float3 I ) const
        {
            return N;
        }
        float3 GetAlbedo( const float3 I ) const
        {
            if ( N.y == 1 )
            {
                // floor albedo: checkerboard
                int ix = (int) ( I.x * 2 + 96.01f );
                int iz = (int) ( I.z * 2 + 96.01f );
                // add deliberate aliasing to two tile
                if ( ix == 98 && iz == 98 ) ix = (int) ( I.x * 32.01f ), iz = (int) ( I.z * 32.01f );
                if ( ix == 94 && iz == 98 ) ix = (int) ( I.x * 64.01f ), iz = (int) ( I.z * 64.01f );
                return float3( ( ( ix + iz ) & 1 ) ? 1 : 0.3f );
            } else if ( N.z == -1 )
            {
                // back wall: logo
                static Surface logo( "assets/logo.png" );
                int ix = (int) ( ( I.x + 4 ) * ( 128.0f / 8 ) );
                int iy = (int) ( ( 2 - I.y ) * ( 64.0f / 3 ) );
                uint p = logo.pixels[( ix & 127 ) + ( iy & 63 ) * 128];
                uint3 i3( ( p >> 16 ) & 255, ( p >> 8 ) & 255, p & 255 );
                return float3( i3 ) * ( 1.0f / 255.0f );
            }
            return float3( 0.93f );
        }
        float GetArea() {
            return 1e30f;
        }
        float3 GetRandomPoint() {
            return float3( 0 ); // TODO: not sure how to approach this
        }
        float3 N;
        float d;
        int objIdx = -1;
    };

    // -----------------------------------------------------------
    // Cube primitive
    // Oriented cube. Unsure if this will also work for rays that
    // start inside it; maybe not the best candidate for testing
    // dielectrics.
    // -----------------------------------------------------------
    class Cube
    {
    public:
        Cube() = default;
        Cube( int idx, float3 pos, float3 size, mat4 transform = mat4::Identity() )
        {
            objIdx = idx;
            b[0] = pos - 0.5f * size, b[1] = pos + 0.5f * size;
            M = transform, invM = transform.FastInvertedTransformNoScale();
        }
        void Intersect( Ray& ray ) const
        {
            // 'rotate' the cube by transforming the ray into object space
            // using the inverse of the cube transform.
            float3 O = TransformPosition( ray.O, invM );
            float3 D = TransformVector( ray.D, invM );
            float rDx = 1 / D.x, rDy = 1 / D.y, rDz = 1 / D.z;
            int signx = D.x < 0, signy = D.y < 0, signz = D.z < 0;
            float tmin = ( b[signx].x - O.x ) * rDx;
            float tmax = ( b[1 - signx].x - O.x ) * rDx;
            float tymin = ( b[signy].y - O.y ) * rDy;
            float tymax = ( b[1 - signy].y - O.y ) * rDy;
            if ( tmin > tymax || tymin > tmax ) return;
            tmin = max( tmin, tymin ), tmax = min( tmax, tymax );
            float tzmin = ( b[signz].z - O.z ) * rDz;
            float tzmax = ( b[1 - signz].z - O.z ) * rDz;
            if ( tmin > tzmax || tzmin > tmax ) return;
            tmin = max( tmin, tzmin ), tmax = min( tmax, tzmax );
            if ( tmin > EPS )
            {
                if ( tmin < ray.t ) ray.t = tmin, ray.objIdx = objIdx, setTextureCoords( ray );
            } else if ( tmax > EPS )
            {
                if ( tmax < ray.t ) ray.t = tmax, ray.objIdx = objIdx, setTextureCoords( ray );
            }
        }
        bool Hit( Ray& ray ) {
            // 'rotate' the cube by transforming the ray into object space
            // using the inverse of the cube transform.
            float3 O = TransformPosition( ray.O, invM );
            float3 D = TransformVector( ray.D, invM );
            float rDx = 1 / D.x, rDy = 1 / D.y, rDz = 1 / D.z;
            int signx = D.x < 0, signy = D.y < 0, signz = D.z < 0;
            float tmin = ( b[signx].x - O.x ) * rDx;
            float tmax = ( b[1 - signx].x - O.x ) * rDx;
            float tymin = ( b[signy].y - O.y ) * rDy;
            float tymax = ( b[1 - signy].y - O.y ) * rDy;
            if ( tmin > tymax || tymin > tmax ) return false;

            tmin = max( tmin, tymin ), tmax = min( tmax, tymax );
            float tzmin = ( b[signz].z - O.z ) * rDz;
            float tzmax = ( b[1 - signz].z - O.z ) * rDz;
            if ( tmin > tzmax || tzmin > tmax ) return false;

            tmin = max( tmin, tzmin );
            tmax = min( tmax, tzmax );
            return (tmin > EPS && tmin < ray.t) || ( tmax > EPS && tmax < ray.t );
        }
        void setTextureCoords( Ray& ray ) const {
            // transform intersection point to object space
            float3 objI = TransformPosition( ray.IntersectionPoint(), invM );
            float uc, vc;

            float d0 = fabs( objI.x - b[0].x ), d1 = fabs( objI.x - b[1].x );
            float d2 = fabs( objI.y - b[0].y ), d3 = fabs( objI.y - b[1].y );
            float d4 = fabs( objI.z - b[0].z ), d5 = fabs( objI.z - b[1].z );
            float minDist = d0;
            int face = 1;
            uc = objI.z, vc = objI.y;
            if ( d1 < minDist ) uc = -objI.z, vc = objI.y, face = 0, minDist = d1;
            if ( d2 < minDist ) uc = objI.x, vc = objI.z, face = 3, minDist = d2;
            if ( d3 < minDist ) uc = objI.x, vc = -objI.z, face = 2, minDist = d3;
            if ( d4 < minDist ) uc = -objI.x, vc = objI.y, face = 5, minDist = d4;
            if ( d5 < minDist ) uc = objI.x, vc = objI.y, face = 4;

            // Convert range from -1 to 1 to 0 to 1
            uc = -uc, vc = -vc;
            uc = 0.5f * ( uc / b[1].x + 1.0f );
            vc = 0.5f * ( vc / b[1].x + 1.0f );

            switch ( face ) {
                case 0: // right
                    ray.u = 0.25f * ( 2 + uc );
                    ray.v = ( 1.0f / 3.0f ) * ( 1 + vc );
                    break;
                case 1: // left
                    ray.u = 0.25f * ( 0 + uc );
                    ray.v = ( 1.0f / 3.0f ) * ( 1 + vc );
                    break;
                case 2: // top
                    ray.u = 0.25f * ( 1 + uc );
                    ray.v = ( 1.0f / 3.0f ) * vc;
                    break;
                case 3: // bottom
                    ray.u = 0.25f * ( 1 + uc );
                    ray.v = ( 1.0f / 3.0f ) * ( 2 + vc );
                    break;
                case 4: // front
                    ray.u = 0.25f * ( 1 + uc );
                    ray.v = ( 1.0f / 3.0f ) * ( 1 + vc );
                    break;
                case 5: // back
                    ray.u = 0.25f * ( 3 + uc );
                    ray.v = ( 1.0f / 3.0f ) * ( 1 + vc );
                    break;
            }
        }
        float3 GetNormal( const float3 I ) const
        {
            // transform intersection point to object space
            float3 objI = TransformPosition( I, invM );
            // determine normal in object space
            float3 N = float3( -1, 0, 0 );
            float d0 = fabs( objI.x - b[0].x ), d1 = fabs( objI.x - b[1].x );
            float d2 = fabs( objI.y - b[0].y ), d3 = fabs( objI.y - b[1].y );
            float d4 = fabs( objI.z - b[0].z ), d5 = fabs( objI.z - b[1].z );
            float minDist = d0;
            if ( d1 < minDist ) minDist = d1, N.x = 1;
            if ( d2 < minDist ) minDist = d2, N = float3( 0, -1, 0 );
            if ( d3 < minDist ) minDist = d3, N = float3( 0, 1, 0 );
            if ( d4 < minDist ) minDist = d4, N = float3( 0, 0, -1 );
            if ( d5 < minDist ) minDist = d5, N = float3( 0, 0, 1 );
            // return normal in world space
            return TransformVector( N, M );
        }
        float3 GetAlbedo( const float3 I ) const
        {
            return float3( 1, 1, 1 );
        }

        // BVH building stuff
        float3 GetCentroid() const {
            return TransformPosition( ( b[0] + b[1] ) * 0.5f, M );
        }

        float3 GetAABBMin() const {
            float3 minPoint = TransformPosition( b[0], M );
            minPoint = fminf( minPoint, TransformPosition( float3( b[1].x, b[0].y, b[0].z ), M ) );
            minPoint = fminf( minPoint, TransformPosition( float3( b[0].x, b[1].y, b[0].z ), M ) );
            minPoint = fminf( minPoint, TransformPosition( float3( b[0].x, b[0].y, b[1].z ), M ) );

            minPoint = fminf( minPoint, TransformPosition( b[1], M ) );
            minPoint = fminf( minPoint, TransformPosition( float3( b[0].x, b[1].y, b[1].z ), M ) );
            minPoint = fminf( minPoint, TransformPosition( float3( b[1].x, b[0].y, b[1].z ), M ) );
            minPoint = fminf( minPoint, TransformPosition( float3( b[1].x, b[1].y, b[0].z ), M ) );
            return minPoint;
        }

        float3 GetAABBMax() const {
            float3 maxPoint = TransformPosition( b[0], M );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[1].x, b[0].y, b[0].z ), M ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[0].x, b[1].y, b[0].z ), M ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[0].x, b[0].y, b[1].z ), M ) );

            maxPoint = fmaxf( maxPoint, TransformPosition( b[1], M ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[0].x, b[1].y, b[1].z ), M ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[1].x, b[0].y, b[1].z ), M ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( b[1].x, b[1].y, b[0].z ), M ) );
            return maxPoint;
        }

        float GetArea() {
            float size = b[1].x - b[0].x;
            return 6.0f * size * size;
        }

        float3 GetRandomPoint() const {
            float size = b[1].x - b[0].x;
            float u = Rand( 2.0f * size ) - size;
            float v = Rand( 2.0f * size ) - size;
            float faceSign = RandomFloat();
            float3 point = float3( 0 );
            if ( faceSign < 0.166666666667f ) {
                point = float3( -size, u, v );
            } else if ( faceSign < 0.333333333333f ) {
                point = float3(  size, u, v );
            } else if ( faceSign < 0.5f ) {
                point = float3( u, -size, v );
            } else if ( faceSign < 0.666666666667f ) {
                point = float3( u,  size, v );
            } else if ( faceSign < 0.833333333333f ) {
                point = float3( u, v, -size );
            } else {
                point = float3( u, v,  size );
            }

            return TransformPosition( point, M );
        }

        float3 b[2];
        mat4 M, invM;
        int objIdx = -1;
    };

    // -----------------------------------------------------------
    // Quad primitive
    // Oriented quad, intended to be used as a light source.
    // -----------------------------------------------------------
    class Quad {
    public:
        Quad() = default;
        Quad( int idx, float s, mat4 transform = mat4::Identity() )
        {
            objIdx = idx;
            size = s * 0.5f;
            T = transform, invT = transform.FastInvertedTransformNoScale();
        }
        void Intersect( Ray& ray ) const
        {
            const float3 O = TransformPosition( ray.O, invT );
            const float3 D = TransformVector( ray.D, invT );
            const float t = O.y / -D.y;
            if ( t < ray.t && t > EPS )
            {
                float3 I = O + t * D;
                if ( I.x > -size && I.x < size && I.z > -size && I.z < size )
                    ray.t = t, ray.objIdx = objIdx;
            }
        }
        bool Hit( Ray& ray ) {
            const float3 O = TransformPosition( ray.O, invT );
            const float3 D = TransformVector( ray.D, invT );
            const float t = O.y / -D.y;
            if ( t < ray.t && t > EPS ) {
                float3 I = O + t * D;
                return I.x > -size && I.x < size&& I.z > -size && I.z < size;
            }

            return false;
        }
        float3 GetNormal( const float3 I ) const
        {
            return TransformVector( float3( 0, -1, 0 ), T );
            //return float3( -T.cell[1], -T.cell[5], -T.cell[9] );
        }
        float3 GetAlbedo( const float3 I ) const
        {
            return float3( 10 );
        }

        // BVH building stuff
        float3 GetCentroid() const {
            return TransformPosition( float3( 0 ), T );
        }

        float3 GetAABBMin() const {
            float3 minPoint = TransformPosition( float3( -size, 0, -size ), T );
            minPoint = fminf( minPoint, TransformPosition( float3( -size, 0, size ), T ) );
            minPoint = fminf( minPoint, TransformPosition( float3( size, 0, -size ), T ) );
            minPoint = fminf( minPoint, TransformPosition( float3( size, 0, size ), T ) );
            return minPoint;
        }

        float3 GetAABBMax() const {
            float3 maxPoint = TransformPosition( float3( -size, 0, -size ), T );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( -size, 0, size ), T ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( size, 0, -size ), T ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( size, 0, size ), T ) );
            return maxPoint;
        }

        float GetArea() const {
            return size * size;
        }

        float3 GetRandomPoint() const {
            float3 randPoint = float3( size * ( RandomFloat() - 1.0f ), size * ( RandomFloat() - 1.0f ), 0.0f );
            return TransformPosition( randPoint, T );
        }

        float size;
        mat4 T, invT;
        int objIdx = -1;
    };

    class Triangle {
    public:
        Triangle() = default;
        Triangle( int idx, float3 v0, float3 v1, float3 v2 ) {
            objIdx = idx;
            A = v0;
            B = v1;
            C = v2;
            this->N = normalize( cross( C - A, B - A ) );
            centroid = ( A + B + C ) / 3;
        }

        void Intersect( Ray& ray ) const {
            float3 AB = B - A;
            float3 AC = C - A;

            float denom = dot( cross( ray.D, AC ), AB );
            // if denom is effectively 0 there is no intersection
            if ( abs( denom ) < CL_DBL_EPSILON ) return;
            denom = 1.0f / denom;

            // we need u in [0, 1]
            float3 AO = ray.O - A;
            float u = dot( cross( -ray.D, AO ), AC ) * denom;
            if ( u < 0 || u > 1 ) return;

            // same for v and (u + v)
            float v = dot( cross( -ray.D, AB ), AO ) * denom;
            if ( v < 0 || u + v > 1 ) return;

            float t = dot( cross( AO, AB ), AC ) * denom;
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                ray.u = u;
                ray.v = v;
            }
        }
        bool Hit( Ray& ray ) {
            float3 AB = B - A;
            float3 AC = C - A;

            float denom = dot( cross( ray.D, AC ), AB );
            // if denom is effectively 0 there is no intersection
            if ( abs( denom ) < CL_DBL_EPSILON ) return false;
            denom = 1.0f / denom;

            // we need u in [0, 1]
            float3 AO = ray.O - A;
            float u = dot( cross( -ray.D, AO ), AC ) * denom;
            if ( u < 0 || u > 1 ) return false;

            // same for v and (u + v)
            float v = dot( cross( -ray.D, AB ), AO ) * denom;
            if ( v < 0 || u + v > 1 ) return false;

            float t = dot( cross( AO, AB ), AC ) * denom;
            return t < ray.t&& t > EPS;
        }

        float3 GetNormal( const float3 I ) const {
            return this->N;
        }

        float3 GetAlbedo( const float3 I ) const {
            return float3( 10 );
        }

        // BVH building stuff
        float3 GetCentroid() const {
            return centroid;
        }

        float3 GetAABBMin() const {
            return fminf( A, fminf( B, C ) );
        }

        float3 GetAABBMax() const {
            return fmaxf( A, fmaxf( B, C ) );
        }

        float GetArea() const {
            float3 AB = B - A;
            float3 AC = C - A;
            return 0.5f * length( cross( AB, AC ) );
        }

        float3 GetRandomPoint() const {
            float u = RandomFloat();
            float v = RandomFloat();
            while ( ( u + v ) > 1 ) {
                u = RandomFloat();
                v = RandomFloat();
            }

            return A + u * B + v * C;
        }

        float3 N;
        float3 A;
        float3 B;
        float3 C;
        float3 centroid;
        int objIdx = -1;
    };

    class ObjModel {
    private:
        uint firstObjIdx;
        vector<float3> vertices;
        vector<Triangle> triangles;
    public:
        ObjModel() = default;
        ObjModel( const string fileName, uint& objIdx, mat4 transform = mat4::Identity() ) {
            ifstream in( fileName, ios::in );
            if ( !in ) {
                printf( "Couldn't open OBJ file.\n" );
                return;
            }

            string line;
            firstObjIdx = objIdx;
            while ( getline( in, line ) ) {
                if ( line.substr( 0, 2 ) == "v " ) {
                    //check v for vertices
                    istringstream v( line.substr( 2 ) );
                    float3 vert;
                    float x, y, z;
                    v >> x; v >> y; v >> z;
                    vert = float3( x, y, z );
                    // vertices.push_back( rotate.TransformPoint( vert * scale + offset ) );
                    vertices.push_back( TransformPosition( vert, transform ) );
                } else if ( line.substr( 0, 2 ) == "f " ) {
                    //check f for faces
                    int a, b, c;
                    const char* chh = line.c_str();

                    sscanf( chh, "f %i/%*i/%*i %i/%*i/%*i %i/%*i/%*i", &a, &b, &c );

                    triangles.push_back( Triangle( objIdx, vertices[a - 1], vertices[b - 1], vertices[c - 1] ) );
                    objIdx++;
                }
            }

            vertices.clear();
        }

        void Intersect( Ray& ray ) const {
            // intersect all triangles in this object
            for ( Triangle t : triangles ) {
                t.Intersect( ray );
            }
        }

        void Intersect( Ray& ray, const int idx ) const {
            triangles[idx].Intersect( ray );
        }

        bool Hit( Ray& ray, const int idx ) {
            return triangles[idx].Hit( ray );
        }

        int hasObject( const int objIdx ) const {
            uint lastObjIdx = firstObjIdx + triangles.size();
            if ( objIdx >= firstObjIdx && objIdx < lastObjIdx ) return objIdx - firstObjIdx;

            return -1;
        }

        float3 GetNormal( int triangle, float3 I ) const {
            //printf("getting triangle %d\n", triangle);
            return triangles[triangle].GetNormal( I );
        }

        // BVH building stuff
        float3 GetCentroid( const int idx ) const {
            return triangles[idx].GetCentroid();
        }

        float3 GetAABBMin( const int idx ) const {
            return triangles[idx].GetAABBMin();
        }

        float3 GetAABBMax( const int idx ) const {
            return triangles[idx].GetAABBMax();
        }

        float GetArea() const {
            float result = 0;
            for ( Triangle triangle : triangles ) {
                result += triangle.GetArea();
            }

            return result;
        }

        float3 GetRandomPoint() const {
            int triangle = (int) ( RandomFloat() * ( triangles.size() - 1 ) );
            return triangles[triangle].GetRandomPoint();
        }
    };

    // -----------------------------------------------------------
    // Scene class
    // We intersect this. The query is internally forwarded to the
    // list of primitives, so that the nearest hit can be returned.
    // For this hit (distance, obj id), we can query the normal and
    // albedo.
    // -----------------------------------------------------------
    class Scene
    {
    public:
        Scene()
        {
            // make the Materials
            red = make_shared<Diffuse>( Diffuse( float3( 0.95f, 0.05f, 0.05f ) ) );
            green = make_shared<Diffuse>( Diffuse( float3( 0.05f, 0.95f, 0.05f ) ) );
            blue = make_shared<Diffuse>( Diffuse( float3( 0.05f, 0.05f, 0.95f ) ) );
            white = make_shared<Diffuse>( Diffuse( float3( 0.95f, 0.95f, 0.95f ) ) );

            mirror = make_shared<Mirror>( Mirror( float3( 0.9f, 0.9f, 0.9f ) ) );

            mix = make_shared<DSMix>( DSMix( float3( 0.9f, 0.2f, 0.1f ), 0.5f ) );

            checkerboard = make_shared<Checkerboard>( Checkerboard( float3( 0.1f, 0.1f, 0.1f ), float3( 0.9f, 0.9f, 0.9f ), 0.95f ) );

            glass = make_shared<Dielectric>( Dielectric( float3( 0.5f, 0.5f, 0.5f ), 1.52f ) );
            diamond = make_shared<Dielectric>( Dielectric( float3( 2.0f, 0.5f, 0.7f ), 2.42f ) );

            lamp = make_shared<Light>( Light( float3( 24.0f, 24.0f, 22.0f ), float3( 0.0f, -1.0f, 0.0f ) ) );

            earth = make_shared<TextureMaterial>( TextureMaterial( "assets/earth.png" ) );

            // we store all primitives in one continuous buffer
            quad = Quad( primitiveCount++, 2 );									// 0: light source
            sphere = Sphere( primitiveCount++, float3( 0 ), 0.5f );				// 1: bouncing ball
            //sphere2 = Sphere( primitiveCount++, float3( 0, 2.5f, -3.07f ), 8 );	// 2: rounded corners
            cube = Cube( primitiveCount++, float3( 0 ), float3( 1.15f ) );			// 3: cube
            triangle = Triangle( primitiveCount++, float3( 0, 0, 3 ), float3( 0.5, -1, 3 ), float3( -0.5, -1, 3 ) ); // 4: triangle
            groundQuad = Quad( primitiveCount++, 50, mat4::Translate( float3( 0.0f, -1.0f, 0.0f ) ) ); // 5: ground quad

            mat4 tetTransform = 
                mat4::Translate( float3( 0, 0.5f, 0.5f ) ) * 
                mat4::RotateX( 0.5 * PI ) * mat4::RotateY( 0.75 * PI ) * mat4::RotateZ( 0.25 * PI ) * 
                mat4::Scale( 0.01f );
            tet = ObjModel( "assets/tetrahedron.obj", primitiveCount, tetTransform );
            mat4 teapotTransform = mat4::Translate( float3( 0, 0.5f, -4.0f ) );// * mat4::Scale( 0.01f );
            teapot = ObjModel( "assets/teapot.obj", primitiveCount, teapotTransform );

            SetTime( 0 );

            // build BVH after objects are moved with setTime
            printf("Building BVH...\n");
            primitiveIndices = (uint*) MALLOC64( primitiveCount * sizeof( uint ) );
            for ( uint i = 0; i < primitiveCount; i++ ) {
                primitiveIndices[i] = i;
            }

            bvhNode = (BVHNode*) MALLOC64( ( 2 * primitiveCount + 1 ) * sizeof( BVHNode ) );
            BuildBVH();
            printf( "Finished BVH!\n" );

            // planes are annoying
            // plane[0] = Plane( primitiveCount + 1, float3( 1, 0, 0 ), 3 );			// 10000: left wall
            // plane[1] = Plane( primitiveCount + 2, float3( -1, 0, 0 ), 2.99f );		// 10001: right wall
            // plane[2] = Plane( primitiveCount + 3, float3( 0, 1, 0 ), 1 );			// 10002: floor
            // plane[3] = Plane( primitiveCount + 4, float3( 0, -1, 0 ), 2 );			// 10003: ceiling
            // plane[4] = Plane( primitiveCount + 5, float3( 0, 0, 1 ), 3 );			// 10004: front wall
            // plane[5] = Plane( primitiveCount + 6, float3( 0, 0, -1 ), 3.99f );		// 10005: back wall
            // Note: once we have triangle support we should get rid of the class
            // hierarchy: virtuals reduce performance somewhat.
        }

        void BuildBVH() {
            BVHNode& root = bvhNode[rootNodeIdx];
            root.leftFirst = 0;
            root.primitiveCount = primitiveCount;
            nodesUsed = 2; // skip a node because memory alignment

            UpdateNodeBounds( rootNodeIdx );
            Subdivide( rootNodeIdx );
        }

        float3 GetCentroid( int objIdx ) {
            float3 centroid = float3( 0 );
            if ( objIdx == quad.objIdx ) centroid = quad.GetCentroid();
            else if ( objIdx == sphere.objIdx ) centroid = sphere.GetCentroid();
            else if ( objIdx == cube.objIdx ) centroid = cube.GetCentroid();
            else if ( objIdx == triangle.objIdx ) centroid = triangle.GetCentroid();
            else if ( objIdx == groundQuad.objIdx ) centroid = groundQuad.GetCentroid();
            else if ( int tetIdx = tet.hasObject( objIdx ); tetIdx != -1 ) centroid = tet.GetCentroid( tetIdx );
            else if ( int teapotIdx = teapot.hasObject( objIdx ); teapotIdx != -1 ) centroid = teapot.GetCentroid( teapotIdx );

            return centroid;
        }

        float3 GetAABBMin( int objIdx ) {
            float3 aabbMin = float3( 0 );
            if ( objIdx == quad.objIdx ) aabbMin = quad.GetAABBMin();
            else if ( objIdx == sphere.objIdx ) aabbMin = sphere.GetAABBMin();
            else if ( objIdx == cube.objIdx ) aabbMin = cube.GetAABBMin();
            else if ( objIdx == triangle.objIdx ) aabbMin = triangle.GetAABBMin();
            else if ( objIdx == groundQuad.objIdx ) aabbMin = groundQuad.GetAABBMin();
            else if ( int tetIdx = tet.hasObject( objIdx ); tetIdx != -1 ) aabbMin = tet.GetAABBMin( tetIdx );
            else if ( int teapotIdx = teapot.hasObject( objIdx ); teapotIdx != -1 ) aabbMin = teapot.GetAABBMin( teapotIdx );

            return aabbMin;
        }

        float3 GetAABBMax( int objIdx ) {
            float3 aabbMax = float3( 0 );
            if ( objIdx == quad.objIdx ) aabbMax = quad.GetAABBMax();
            else if ( objIdx == sphere.objIdx ) aabbMax = sphere.GetAABBMax();
            else if ( objIdx == cube.objIdx ) aabbMax = cube.GetAABBMax();
            else if ( objIdx == triangle.objIdx ) aabbMax = triangle.GetAABBMax();
            else if ( objIdx == groundQuad.objIdx ) aabbMax = groundQuad.GetAABBMax();
            else if ( int tetIdx = tet.hasObject( objIdx ); tetIdx != -1 ) aabbMax = tet.GetAABBMax( tetIdx );
            else if ( int teapotIdx = teapot.hasObject( objIdx ); teapotIdx != -1 ) aabbMax = teapot.GetAABBMax( teapotIdx );

            return aabbMax;
        }

        float GetArea( int objIdx ) {
            float area = 0.0f;
            if ( objIdx == quad.objIdx ) area = quad.GetArea();
            else if ( objIdx == sphere.objIdx ) area = sphere.GetArea();
            else if ( objIdx == cube.objIdx ) area = cube.GetArea();
            else if ( objIdx == triangle.objIdx ) area = triangle.GetArea();
            else if ( objIdx == groundQuad.objIdx ) area = groundQuad.GetArea();
            else if ( tet.hasObject( objIdx ) != -1 ) area = tet.GetArea();
            else if ( teapot.hasObject( objIdx ) != -1 ) area = teapot.GetArea();

            return area;
        }

        void Subdivide( uint nodeIdx ) {
            BVHNode& node = bvhNode[nodeIdx];
            // determine split axis and position
            float3 extent = node.aabbMax - node.aabbMin;

            // determine split axis using SAH
            int axis;
            float splitPos;
            float bestCost = FindBestSplitPlane( node, axis, splitPos );
            float parentCost = calculateNodeCost( node );
            if ( bestCost >= parentCost ) return;

            // in-place partition
            int i = node.leftFirst;
            int j = i + node.primitiveCount - 1;
            while ( i <= j ) {
                int objIdx = primitiveIndices[node.leftFirst + i];
                float3 centroid = GetCentroid( objIdx );
                if ( centroid[axis] < splitPos ) i++;
                else swap( primitiveIndices[i], primitiveIndices[j--] );
            }
            // abort split if one of the sides is empty
            int leftCount = i - node.leftFirst;
            if ( leftCount == 0 || leftCount == node.primitiveCount ) return;
            // create child nodes
            int leftChildIdx = nodesUsed++;
            int rightChildIdx = nodesUsed++;
            bvhNode[leftChildIdx].leftFirst = node.leftFirst;
            bvhNode[leftChildIdx].primitiveCount = leftCount;
            bvhNode[rightChildIdx].leftFirst = i;
            bvhNode[rightChildIdx].primitiveCount = node.primitiveCount - leftCount;
            node.leftFirst = leftChildIdx;
            node.primitiveCount = 0;
            UpdateNodeBounds( leftChildIdx );
            UpdateNodeBounds( rightChildIdx );
            // recurse
            Subdivide( leftChildIdx );
            Subdivide( rightChildIdx );
        }

        float FindBestSplitPlane( BVHNode& node, int& axis, float& splitPos ) {
            float bestCost = 1e30f;
            for ( int a = 0; a < 3; a++ ) {
                float boundsMin = 1e30f;
                float boundsMax = -1e30f;
                for ( int i = 0; i < node.primitiveCount; i++ ) {
                    float3 centroid = GetCentroid( primitiveIndices[node.leftFirst + i] );
                    boundsMin = min( boundsMin, centroid[a] );
                    boundsMax = max( boundsMax, centroid[a] );
                }

                if ( boundsMin == boundsMax ) continue;

                // populate the bins
                BVHBin bin[BIN_COUNT];
                float scale = BIN_COUNT / ( boundsMax - boundsMin );
                for ( uint i = 0; i < node.primitiveCount; i++ ) {
                    int objIdx = primitiveIndices[node.leftFirst + i];
                    float centroidAtAxis = GetCentroid( objIdx )[a];
                    int binIdx = min( BIN_COUNT - 1, (int) ( ( centroidAtAxis - boundsMin ) * scale ) );
                    // update the found bin with the bounding box needed for the found object
                    bin[binIdx].primitiveCount++;
                    bin[binIdx].bounds.Grow( GetAABBMin( objIdx ) );
                    bin[binIdx].bounds.Grow( GetAABBMax( objIdx ) );
                }

                // gather data for the planes between the bins
                aabb leftBox;
                aabb rightBox;
                int leftCount[BIN_COUNT - 1];
                int rightCount[BIN_COUNT - 1];
                int leftSum = 0;
                int rightSum = 0;
                float leftArea[BIN_COUNT - 1];
                float rightArea[BIN_COUNT - 1];
                for ( int i = 0; i < BIN_COUNT - 1; i++ ) {
                    // fill in data for the left
                    leftSum += bin[i].primitiveCount;
                    leftCount[i] = leftSum;
                    leftBox.Grow( bin[i].bounds );
                    leftArea[i] = leftBox.Area();
                    // same for data on the right
                    rightSum += bin[BIN_COUNT - 1 - i].primitiveCount;
                    rightCount[BIN_COUNT - 2 - i] = rightSum;
                    rightBox.Grow( bin[BIN_COUNT - 1 - i].bounds );
                    rightArea[BIN_COUNT - 2 - i] = rightBox.Area();
                }

                // calculate SAH cost for the planes
                scale = ( boundsMax - boundsMin ) / BIN_COUNT;
                for ( int i = 0; i < BIN_COUNT - 1; i++ ) {
                    float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];

                    if ( planeCost < bestCost ) {
                        axis = a;
                        splitPos = boundsMin + scale * ( i + 1 );
                        bestCost = planeCost;
                    }
                }
            }

            return bestCost;
        }

        float calculateNodeCost( BVHNode& node ) {
            float3 e = node.aabbMax - node.aabbMin;
            float area = e.x * e.y + e.y * e.z + e.z * e.x;
            return node.primitiveCount * area;
        }

        void UpdateNodeBounds( uint nodeIdx ) {
            BVHNode& node = bvhNode[nodeIdx];
            node.aabbMin = float3( 1e30f );
            node.aabbMax = float3( -1e30f );
            uint first = node.leftFirst;
            for ( int i = 0; i < node.primitiveCount; i++ ) {
                uint objIdx = primitiveIndices[first + i];
                node.aabbMin = fminf( node.aabbMin, GetAABBMin( objIdx ) );
                node.aabbMax = fmaxf( node.aabbMax, GetAABBMax( objIdx ) );
            }
        }

        shared_ptr<ObjectMaterial> GetMaterial( int objIdx ) {
            if ( objIdx == quad.objIdx ) return lamp;
            else if ( objIdx == sphere.objIdx ) return earth;
            else if ( objIdx == cube.objIdx ) return diamond;
            else if ( objIdx == triangle.objIdx ) return mix;
            else if ( objIdx == groundQuad.objIdx ) return checkerboard;
            else if ( tet.hasObject( objIdx ) != -1 ) return mirror;
            else if ( teapot.hasObject( objIdx ) != -1 ) return white;

            return white;
        }
        void SetTime( float t )
        {
            // default time for the scene is simply 0. Updating/ the time per frame 
            // enables animation. Updating it per ray can be used for motion blur.
            animTime = t;
            // light source animation: swing
            mat4 M1base = mat4::Translate( float3( 0, 2.6f, 2 ) );
            mat4 M1 = M1base * mat4::RotateZ( sinf( animTime * 0.6f ) * 0.1f ) * mat4::Translate( float3( 0, -0.9, 0 ) );
            quad.T = M1, quad.invT = M1.FastInvertedTransformNoScale();
            // cube animation: spin
            mat4 M2base = mat4::RotateX( PI / 4 ) * mat4::RotateZ( PI / 4 );
            mat4 M2 = mat4::Translate( float3( 1.4f, 0, 2 ) ) * mat4::RotateY( animTime * 0.5f ) * M2base;
            cube.M = M2, cube.invM = M2.FastInvertedTransformNoScale();
            // sphere animation: bounce
            float tm = 1 - sqrf( fmodf( animTime, 2.0f ) - 1 );
            sphere.pos = float3( -1.4f, -0.5f + tm, 2 );
        }
        int GetRandomLight() const {
            return 0; // only one light atm...
        }
        float3 GetLightColor( int objIdx, Ray& ray ) {
            return GetMaterial( objIdx )->GetColor( ray );
        }
        float3 GetLightPos( int objIdx ) const {
            if ( objIdx == quad.objIdx ) return quad.GetRandomPoint();
            else if ( objIdx == sphere.objIdx ) return sphere.GetRandomPoint();
            else if ( objIdx == cube.objIdx ) return cube.GetRandomPoint();
            else if ( objIdx == triangle.objIdx ) return triangle.GetRandomPoint();
            else if ( objIdx == groundQuad.objIdx ) return groundQuad.GetRandomPoint();
            else if ( tet.hasObject( objIdx ) != -1 ) return tet.GetRandomPoint();
            else if ( teapot.hasObject( objIdx ) != -1 ) return teapot.GetRandomPoint();
        }
        float3 GetLightPos() const
        {
            // random point on the quad area
            float3 randomPoint = TransformPosition(float3(-0.5f, 0, -0.5f) + Rand(1) * float3(1, 0, 0) + Rand(1) * float3(0, 0, 1), quad.T);
            return randomPoint - float3( 0, 0.1f, 0 );
        }
        float3 GetLightColor() const {
            return float3( 24, 24, 22 );
        }
        float3 GetLightDir() const {
            return float3( 0.0f, -1.0f, 0.0f );
        }
        void FindNearest( Ray& ray ) const {
            // room walls - ugly shortcut for more speed
            float t;
            //if ( ray.D.x < 0 ) PLANE_X( 3, 4 ) else PLANE_X( -2.99f, 5 );
            //if ( ray.D.y < 0 ) PLANE_Y( 1, 6 ) else PLANE_Y( -2, 7 );
            //if ( ray.D.z < 0 ) PLANE_Z( 3, 8 ) else PLANE_Z( -3.99f, 9 );
            //plane[2].Intersect( ray );
            //plane[5].Intersect( ray );
            quad.Intersect( ray );
            groundQuad.Intersect( ray );
            sphere.Intersect( ray );
            cube.Intersect( ray );
            triangle.Intersect(ray);
            tet.Intersect(ray);
            teapot.Intersect( ray );
        }

        void IntersectBVH( Ray& ray ) {
            // start at the root
            BVHNode* node = &bvhNode[rootNodeIdx];

            // stack stuff
            uint stackPtr = 0;
            BVHNode* stack[64];
            while ( true ) {
                if ( node->isLeaf() ) {
                    for ( uint i = 0; i < node->primitiveCount; i++ ) {
                        int objIdx = primitiveIndices[node->leftFirst + i];
                        if ( objIdx == quad.objIdx ) quad.Intersect( ray );
                        else if ( objIdx == sphere.objIdx ) sphere.Intersect( ray );
                        else if ( objIdx == cube.objIdx ) cube.Intersect( ray );
                        else if ( objIdx == triangle.objIdx ) triangle.Intersect( ray );
                        else if ( objIdx == groundQuad.objIdx ) groundQuad.Intersect( ray );
                        else if ( int tetIdx = tet.hasObject( objIdx ); tetIdx != -1 ) tet.Intersect( ray, tetIdx );
                        else if ( int teapotIdx = teapot.hasObject( objIdx ); teapotIdx != -1 ) teapot.Intersect( ray, teapotIdx );
                    }

                    // stop the loop if we are at stack pointer 0, aka the root
                    if ( stackPtr == 0 ) break;

                    node = stack[--stackPtr];
                    continue;
                }

                BVHNode* child1 = &bvhNode[node->leftFirst];
                BVHNode* child2 = &bvhNode[node->leftFirst + 1];

                float dist1 = IntersectAABB( ray, child1->aabbMin, child1->aabbMax );
                float dist2 = IntersectAABB( ray, child2->aabbMin, child2->aabbMax );
                if ( dist1 > dist2 ) { 
                    swap( dist1, dist2 );
                    swap( child1, child2 ); 
                }

                if ( dist1 == 1e30f ) {
                    if ( stackPtr == 0 ) break;
                    else node = stack[--stackPtr];
                } else {
                    node = child1;
                    if ( dist2 != 1e30f ) stack[stackPtr++] = child2;
                }
            }
        }

        float IntersectAABB( const Ray& ray, const float3 bmin, const float3 bmax ) {
            float tx1 = ( bmin.x - ray.O.x ) * ray.rD.x;
            float tx2 = ( bmax.x - ray.O.x ) * ray.rD.x;
            float tmin = min( tx1, tx2 );
            float tmax = max( tx1, tx2 );

            float ty1 = ( bmin.y - ray.O.y ) * ray.rD.y;
            float ty2 = ( bmax.y - ray.O.y ) * ray.rD.y;
            tmin = max( tmin, min( ty1, ty2 ) );
            tmax = min( tmax, max( ty1, ty2 ) );

            float tz1 = ( bmin.z - ray.O.z ) * ray.rD.z;
            float tz2 = ( bmax.z - ray.O.z ) * ray.rD.z;
            tmin = max( tmin, min( tz1, tz2 ) );
            tmax = min( tmax, max( tz1, tz2 ) );

            if ( tmax >= tmin && tmin < ray.t && tmax > 0 ) return tmin;
            return 1e30f;
        }

        bool IsOccludedOld( Ray& ray ) const {
            float rayLength = ray.t;
            // skip planes: it is not possible for the walls to occlude anything
            quad.Intersect( ray );
            groundQuad.Intersect( ray );
            sphere.Intersect( ray );
            //sphere2.Intersect( ray );
            cube.Intersect( ray );
            triangle.Intersect( ray );
            tet.Intersect(ray);
            teapot.Intersect( ray );
            return ray.t < rayLength && ray.t > EPS; // final addition to compensate for floating point inaccuracy
            // technically this is wasteful: 
            // - we potentially search beyond rayLength
            // - we store objIdx and t when we just need a yes/no
            // - we don't 'early out' after the first occlusion
        }

        bool IsOccluded( Ray& ray ) {
            float rayLength = ray.t;

            BVHNode* node = &bvhNode[rootNodeIdx], * stack[64];
            uint stackPtr = 0;
            bool isOccluded = false;
            while ( true ) {
                if ( node->isLeaf() ) {
                    for ( uint i = 0; i < node->primitiveCount; i++ ) {
                        int objIdx = primitiveIndices[node->leftFirst + i];
                        if ( objIdx == quad.objIdx ) isOccluded = quad.Hit( ray );
                        else if ( objIdx == sphere.objIdx ) isOccluded = sphere.Hit( ray );
                        else if ( objIdx == cube.objIdx ) isOccluded = cube.Hit( ray );
                        else if ( objIdx == triangle.objIdx ) isOccluded = triangle.Hit( ray );
                        else if ( objIdx == groundQuad.objIdx ) isOccluded = groundQuad.Hit( ray );
                        else if ( int tetIdx = tet.hasObject( objIdx ); tetIdx != -1 ) isOccluded = tet.Hit( ray, tetIdx );
                        else if ( int teapotIdx = teapot.hasObject( objIdx ); teapotIdx != -1 ) isOccluded = teapot.Hit( ray, teapotIdx );

                        if ( isOccluded ) return true;
                    }

                    if ( stackPtr == 0 ) break;
                    else node = stack[--stackPtr];
                    continue;
                }

                BVHNode* child1 = &bvhNode[node->leftFirst];
                BVHNode* child2 = &bvhNode[node->leftFirst + 1];
                float dist1 = IntersectAABB( ray, child1->aabbMin, child1->aabbMax );
                float dist2 = IntersectAABB( ray, child2->aabbMin, child2->aabbMax );
                if ( dist1 > dist2 ) {
                    swap( dist1, dist2 );
                    swap( child1, child2 );
                }

                if ( dist1 > 1e30f - FLT_EPSILON ) {
                    if ( stackPtr == 0 ) break;
                    else node = stack[--stackPtr];
                } else {
                    node = child1;
                    if ( dist2 < 1e30f ) stack[stackPtr++] = child2;
                }
            }

            return false;
        }

        float3 GetNormal( int objIdx, float3 I, float3 wo ) const
        {
            // we get the normal after finding the nearest intersection:
            // this way we prevent calculating it multiple times.
            if ( objIdx == -1 ) return float3( 0 ); // or perhaps we should just crash
            float3 N;
            if ( objIdx == quad.objIdx ) N = quad.GetNormal( I );
            else if ( objIdx == sphere.objIdx ) N = sphere.GetNormal( I );
            else if ( objIdx == cube.objIdx ) N = cube.GetNormal( I );
            else if ( objIdx == triangle.objIdx ) N = triangle.GetNormal( I );
            else if ( objIdx == groundQuad.objIdx ) N = groundQuad.GetNormal( I );
            else if( int modelTriangle = tet.hasObject( objIdx ); modelTriangle != -1 ) N = tet.GetNormal( modelTriangle, I );
            else if ( int modelTriangle = teapot.hasObject( objIdx ); modelTriangle != -1 ) N = teapot.GetNormal( modelTriangle, I );
            else {
                // faster to handle the 6 planes without a call to GetNormal
                N = float3( 0 );
                N[( objIdx - 4 ) / 2] = 1 - 2 * (float) ( objIdx & 1 );
            }
            if ( dot( N, wo ) > 0 ) N = -N; // hit backside / inside
            return N;
        }
        __declspec( align( 64 ) ) // start a new cacheline here
            float animTime = 0;
        Quad quad;
        Quad groundQuad;
        Sphere sphere;
        Sphere sphere2;
        Cube cube;
        Triangle triangle;
        Plane plane[6];
        ObjModel tet;
        ObjModel teapot;

        shared_ptr<ObjectMaterial> red;
        shared_ptr<ObjectMaterial> green;
        shared_ptr<ObjectMaterial> blue;
        shared_ptr<ObjectMaterial> white;

        shared_ptr<ObjectMaterial> mirror;

        shared_ptr<ObjectMaterial> mix;

        shared_ptr<ObjectMaterial> checkerboard;

        shared_ptr<ObjectMaterial> glass;
        shared_ptr<ObjectMaterial> diamond;

        shared_ptr<ObjectMaterial> lamp;

        shared_ptr<ObjectMaterial> earth;

        BVHNode* bvhNode;
        uint* primitiveIndices;
        uint rootNodeIdx = 0;
        uint nodesUsed;
        uint primitiveCount = 0;
    };

}