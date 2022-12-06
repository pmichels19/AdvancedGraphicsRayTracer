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
        float3 GetNormal( const float3 I ) const
        {
            return ( I - this->pos ) * invr;
        }
        float3 GetAlbedo( const float3 I ) const
        {
            return float3( 0.93f );
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
        void setTextureCoords( Ray& ray ) const {
            // transform intersection point to object space
            float3 objI = TransformPosition( ray.IntersectionPoint(), invM);
            float uc, vc;

            float d0 = fabs( objI.x - b[0].x ), d1 = fabs( objI.x - b[1].x );
            float d2 = fabs( objI.y - b[0].y ), d3 = fabs( objI.y - b[1].y );
            float d4 = fabs( objI.z - b[0].z ), d5 = fabs( objI.z - b[1].z );
            float minDist = d0;
            int face = 1;
            uc = objI.z, vc = objI.y;
            if ( d1 < minDist ) uc = -objI.z, vc =  objI.y, face = 0, minDist = d1;
            if ( d2 < minDist ) uc =  objI.x, vc =  objI.z, face = 3, minDist = d2;
            if ( d3 < minDist ) uc =  objI.x, vc = -objI.z, face = 2, minDist = d3;
            if ( d4 < minDist ) uc = -objI.x, vc =  objI.y, face = 5, minDist = d4;
            if ( d5 < minDist ) uc =  objI.x, vc =  objI.y, face = 4;

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
        float3 GetNormal( const float3 I ) const
        {
            // TransformVector( float3( 0, -1, 0 ), T ) 
            return float3( -T.cell[1], -T.cell[5], -T.cell[9] );
        }
        float3 GetAlbedo( const float3 I ) const
        {
            return float3( 10 );
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
        }

        void Intersect( Ray& ray ) const {
            float3 AB = B - A;
            float3 AC = C - A;

            float denom = dot( cross( ray.D, AC ), AB );
            // if denom is effectively 0 there is no intersection
            if ( abs( denom ) < CL_DBL_EPSILON ) {
                return;
            }

            // we need u in [0, 1]
            float3 AO = ray.O - A;
            float u = dot( cross( -ray.D, AO ), AC ) / denom;
            if ( u < 0 || u > 1 ) return;

            // same for v and (u + v)
            float v = dot( cross( -ray.D, AB ), AO ) / denom;
            if ( v < 0 || u + v > 1 ) return;

            float t = dot( cross( AO, AB ), AC ) / denom;
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                ray.u = u;
                ray.v = v;
            }
        }

        float3 GetNormal( const float3 I ) const {
            return this->N;
        }

        float3 GetAlbedo( const float3 I ) const {
            return float3( 10 );
        }

        float3 N;
        float3 A;
        float3 B;
        float3 C;
        int objIdx = -1;
    };

    class ObjModel {
    private:
        vector<float3> vertices;
        vector<Triangle> triangles;
    public:
        ObjModel() = default;
        ObjModel( const string fileName, int &objIdx, const float scale = 1, const float3 offset = float3( 0 ), mat4 rotate = mat4::Identity() ) {
            ifstream in( fileName, ios::in );
            if ( !in ) {
                printf("Couldn't open OBJ file.\n");
                return;
            }

            string line;
            while ( getline( in, line ) ) {
                if ( line.substr( 0, 2 ) == "v " ) {
                    //check v for vertices
                    istringstream v( line.substr( 2 ) );
                    float3 vert;
                    float x, y, z;
                    v >> x; v >> y; v >> z;
                    vert = float3( x, y, z );
                    vertices.push_back( rotate.TransformPoint( vert * scale + offset ) );
                } else if ( line.substr( 0, 2 ) == "f " ) {
                    //check f for faces
                    int a, b, c;
                    const char* chh = line.c_str();

                    sscanf( chh, "f %i/%*i/%*i %i/%*i/%*i %i/%*i/%*i", &a, &b, &c );

                    triangles.push_back( Triangle( objIdx, vertices[a - 1], vertices[b - 1], vertices[c - 1] ));
                    objIdx++;
                }
            }
        }

        void Intersect( Ray& ray ) const {
            // intersect all triangles in this object
            for ( Triangle t : triangles ) {
                t.Intersect( ray );
            }
        }

        int hasObject( const int objIdx ) const {
            int idx = 0;
            for ( Triangle t : triangles ) {
                if ( t.objIdx == objIdx ) return idx;
                idx++;
            }

            return -1;
        }

        float3 GetNormal( int triangle, float3 I ) const {
            return triangles[triangle].GetNormal( I );
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
            red     = make_shared<Diffuse>( Diffuse( float3( 1.0f, 0.0f, 0.0f ) ) );
            green   = make_shared<Diffuse>( Diffuse( float3( 0.0f, 1.0f, 0.0f ) ) );
            blue    = make_shared<Diffuse>( Diffuse( float3( 0.0f, 0.0f, 1.0f ) ) );
            white   = make_shared<Diffuse>( Diffuse( float3( 1.0f, 1.0f, 1.0f ) ) );

            mirror  = make_shared<Mirror>( Mirror( float3( 1.0f, 1.0f, 1.0f ) ) );

            mix     = make_shared<DSMix>( DSMix( float3( 0.9f, 0.2f, 0.1f ), 0.5f ) );

            checkerboard = make_shared<Checkerboard>( Checkerboard( float3( 0.1f, 0.1f, 0.1f ), float3( 0.9f, 0.9f, 0.9f ), 0.95f ) );

            glass   = make_shared<Dielectric>( Dielectric( float3( 0.5f, 0.5f, 0.5f ), 1.52f ) );
            diamond = make_shared<Dielectric>( Dielectric( float3( 4.0f, 1.0f, 0.7f ), 2.42f ) );

            lamp = make_shared<Light>( Light( float3( 24.0f, 24.0f, 22.0f ), float3( 0.0f, -1.0f, 0.0f ) ) );
            
            earth = make_shared<TextureMaterial>( TextureMaterial( "assets/earth.png" ) );
            // we store all primitives in one continuous buffer
            quad = Quad( 0, 1 );									// 0: light source
            sphere = Sphere( 1, float3( 0 ), 0.5f );				// 1: bouncing ball
            sphere2 = Sphere( 2, float3( 0, 2.5f, -3.07f ), 8 );	// 2: rounded corners
            cube = Cube( 3, float3( 0 ), float3( 1.15f ) );			// 3: cube
            plane[0] = Plane( 4, float3( 1, 0, 0 ), 3 );			// 4: left wall
            plane[1] = Plane( 5, float3( -1, 0, 0 ), 2.99f );		// 5: right wall
            plane[2] = Plane( 6, float3( 0, 1, 0 ), 1 );			// 6: floor
            plane[3] = Plane( 7, float3( 0, -1, 0 ), 2 );			// 7: ceiling
            plane[4] = Plane( 8, float3( 0, 0, 1 ), 3 );			// 8: front wall
            plane[5] = Plane( 9, float3( 0, 0, -1 ), 3.99f );		// 9: back wall
            triangle = Triangle(10, float3( 0, 0, 3 ), float3( 0.5, 1, 3 ), float3( -0.5, 1, 3 ) ); // 10: triangle

            // start at 10000 just so we always have enough
            int modelObjIndices = 10000;
            mat4 tetRotation = mat4::RotateX(0.5 * PI) * mat4::RotateY( 0.75 * PI ) * mat4::RotateZ( 0.25 * PI );
            tet = ObjModel("assets/tetrahedron.obj", modelObjIndices, 1.0f / 100.0f, float3( 0 , 0.5f, 0.5f ), tetRotation );
            SetTime( 0 );
            // Note: once we have triangle support we should get rid of the class
            // hierarchy: virtuals reduce performance somewhat.
        }
        shared_ptr<ObjectMaterial> GetMaterial( int objIdx ) {
            switch ( objIdx ) {
                case 0:     // light panel
                    return lamp;
                case 1:     // bouncing ball
                    return diamond;
                case 2:     // rounded corners
                    return green;
                case 3:     // cube
                    return earth;
                case 4:     // left wall
                    return green;
                case 5:     // right wall
                    return red;
                case 6:     // floor
                    return checkerboard;
                case 7:     // ceiling
                    return white;
                case 8:     // front wall
                    return white;
                case 9:     // back wall
                    return green;
                case 10:    // triangle
                    return mix;
                default:
                    if ( tet.hasObject( objIdx ) != -1 ) {
                        return mirror;
                    }

                    printf( "This should be unreachable - scene, getMaterial()\n" );
                    return white;
            }
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
            //cube.M = M2, cube.invM = M2.FastInvertedTransformNoScale();
            // sphere animation: bounce
            float tm = 1 - sqrf( fmodf( animTime, 2.0f ) - 1 );
            sphere.pos = float3( -1.4f, -0.5f + tm, 2 );
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
        void FindNearest( Ray& ray ) const
        {
            // room walls - ugly shortcut for more speed
            float t;
            //if ( ray.D.x < 0 ) PLANE_X( 3, 4 ) else PLANE_X( -2.99f, 5 );
            //if ( ray.D.y < 0 ) PLANE_Y( 1, 6 ) else PLANE_Y( -2, 7 );
            //if ( ray.D.z < 0 ) PLANE_Z( 3, 8 ) else PLANE_Z( -3.99f, 9 );
            plane[2].Intersect( ray );
            //plane[5].Intersect( ray );
            quad.Intersect( ray );
            sphere.Intersect( ray );
            //sphere2.Intersect( ray );
            cube.Intersect( ray );
            triangle.Intersect(ray);
            //tet.Intersect(ray);
        }
        bool IsOccluded( Ray& ray ) const
        {
            float rayLength = ray.t;
            // skip planes: it is not possible for the walls to occlude anything
            quad.Intersect( ray );
            sphere.Intersect( ray );
            //sphere2.Intersect( ray );
            cube.Intersect( ray );
            triangle.Intersect( ray );
            tet.Intersect(ray);
            return ray.t < rayLength && ray.t > EPS; // final addition to compensate for floating point inaccuracy
            // technically this is wasteful: 
            // - we potentially search beyond rayLength
            // - we store objIdx and t when we just need a yes/no
            // - we don't 'early out' after the first occlusion
        }
        float3 GetNormal( int objIdx, float3 I, float3 wo ) const
        {
            // we get the normal after finding the nearest intersection:
            // this way we prevent calculating it multiple times.
            if ( objIdx == -1 ) return float3( 0 ); // or perhaps we should just crash
            float3 N;
            int modelTriangle = tet.hasObject( objIdx );
            if ( modelTriangle != -1 ) N = tet.GetNormal( modelTriangle, I );
            else if ( objIdx == 0 ) N = quad.GetNormal( I );
            else if ( objIdx == 1 ) N = sphere.GetNormal( I );
            else if ( objIdx == 2 ) N = sphere2.GetNormal( I );
            else if ( objIdx == 3 ) N = cube.GetNormal( I );
            else if ( objIdx == 10 ) N = triangle.GetNormal( I );
            else {
                // faster to handle the 6 planes without a call to GetNormal
                N = float3( 0 );
                N[( objIdx - 4 ) / 2] = 1 - 2 * (float) ( objIdx & 1 );
            }
            if ( dot( N, wo ) > 0 ) N = -N; // hit backside / inside
            return N;
        }
        float3 GetAlbedo( int objIdx, float3 I ) const
        {
            if ( objIdx == -1 ) return float3( 0 ); // or perhaps we should just crash
            if ( objIdx == 0 ) return quad.GetAlbedo( I );
            if ( objIdx == 1 ) return sphere.GetAlbedo( I );
            if ( objIdx == 2 ) return sphere2.GetAlbedo( I );
            if ( objIdx == 3 ) return cube.GetAlbedo( I );
            else if ( objIdx == 10 ) return triangle.GetAlbedo( I );
            return plane[objIdx - 4].GetAlbedo( I );
            // once we have triangle support, we should pass objIdx and the bary-
            // centric coordinates of the hit, instead of the intersection location.
        }
        float GetReflectivity( int objIdx, float3 I ) const
        {
            if ( objIdx == 1 /* ball */ ) return 1;
            if ( objIdx == 6 /* floor */ ) return 0.3f;
            return 0;
        }
        float GetRefractivity( int objIdx, float3 I ) const
        {
            return objIdx == 3 ? 1.0f : 0.0f;
        }
        __declspec( align( 64 ) ) // start a new cacheline here
            float animTime = 0;
        Quad quad;
        Sphere sphere;
        Sphere sphere2;
        Cube cube;
        Triangle triangle;
        Plane plane[6];
        ObjModel tet;

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
    };

}