#pragma once

enum PrimitiveType {
    SPHERE = 0,
    PLANE = 1,
    CUBE = 2,
    QUAD = 3,
    TRIANGLE = 4
};

/**
* Primitive wrapper class
*
* All primitives can be expressed in this class in the given formats:
* SPHERE:
* PLANE:
* CUBE:
* QUAD:
* TRIANGLE:
**/
class Primitive {
public:
    float3 centroid;
    vector<float3> data;

    mat4 Transform;
    mat4 InvertedTransform;

    PrimitiveType type;

    uint objIdx;

    Primitive() = default;
    Primitive( vector<float3> data, PrimitiveType type, uint& idx, mat4 Transform = mat4::Identity() ): data( data ), type( type ), objIdx( idx++ ), Transform( Transform ) {
        // get the inverted transformation matrix
        InvertedTransform = Transform.FastInvertedTransformNoScale();
        // set the centroid
        centroid = float3( 0 );
        switch ( type ) {
            case PLANE:
                centroid = -data[0] * data[1].x; // TODO
                break;
            case TRIANGLE:
                centroid = ( data[0] + data[1] + data[2] ) / 3.0f;
                break;
        }
    }

    /**
    * Update to a new transformation matrix, also immediately set its inverse
    **/
    void UpdateTransform( mat4 newTransform ) {
        Transform = newTransform;
        InvertedTransform = Transform.FastInvertedTransformNoScale();
    }

    /**
    * Intersect this primitive with a ray
    **/
    void Intersect( Ray& ray ) const {
        if ( type == SPHERE ) {
            float3 pos = TransformPosition( float3( 0.0f ), Transform );
            float3 oc = ray.O - pos;
            float b = dot( oc, ray.D );
            float c = dot( oc, oc ) - data[0].y;
            float t, d = b * b - c;

            if ( d <= 0 ) return;

            d = sqrtf( d ), t = -b - d;
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 cToI = normalize( ray.IntersectionPoint() - pos );
                ray.u = 0.5f - atan2f( cToI.z, cToI.x ) * INV2PI;
                ray.v = 0.5f - asinf( cToI.y ) * INVPI;
                return;
            }

            t = d - b;
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 cToI = normalize( ray.IntersectionPoint() - pos );
                ray.u = 0.5f - atan2f( cToI.z, cToI.x ) * INV2PI;
                ray.v = 0.5f - asinf( cToI.y ) * INVPI;
                return;
            }
        } else if ( type == PLANE ) {
            float t = -( dot( ray.O, data[0] ) + data[1].x ) / ( dot( ray.D, data[0] ) );
            if ( t < ray.t && t > EPS ) {
                ray.t = t;
                ray.objIdx = objIdx;
                float3 I = ray.IntersectionPoint();
                if ( data[0].x < FLT_EPSILON && data[0].y < FLT_EPSILON ) {
                    ray.u = I.x;
                    ray.v = -I.y;
                } else if ( data[0].x < FLT_EPSILON && data[0].z < FLT_EPSILON ) {
                    ray.u = I.x;
                    ray.v = -I.z;
                } else if ( data[0].y < FLT_EPSILON && data[0].z < FLT_EPSILON ) {
                    ray.u = I.y;
                    ray.v = -I.z;
                }
            }
        } else if ( type == CUBE ) {
            // 'rotate' the cube by transforming the ray into object space
            // using the inverse of the cube transform.
            float3 O = TransformPosition( ray.O, InvertedTransform );
            float3 D = TransformVector( ray.D, InvertedTransform );

            float rDx = 1 / D.x;
            float rDy = 1 / D.y;
            float rDz = 1 / D.z;

            int signx = D.x < 0;
            int signy = D.y < 0;
            int signz = D.z < 0;

            float tmin = ( data[signx].x - O.x ) * rDx;
            float tmax = ( data[1 - signx].x - O.x ) * rDx;
            float tymin = ( data[signy].y - O.y ) * rDy;
            float tymax = ( data[1 - signy].y - O.y ) * rDy;
            if ( tmin > tymax || tymin > tmax ) return;

            tmin = max( tmin, tymin );
            tmax = min( tmax, tymax );
            float tzmin = ( data[signz].z - O.z ) * rDz;
            float tzmax = ( data[1 - signz].z - O.z ) * rDz;
            if ( tmin > tzmax || tzmin > tmax ) return;

            tmin = max( tmin, tzmin );
            tmax = min( tmax, tzmax );
            if ( tmin > EPS ) {
                if ( tmax < ray.t ) {
                    ray.t = tmin;
                    ray.objIdx = objIdx;
                    setTextureCoordsCube( ray );
                }
            } else if ( tmax > EPS ) {
                if ( tmax < ray.t ) {
                    ray.t = tmax;
                    ray.objIdx = objIdx;
                    setTextureCoordsCube( ray );
                }
            }
        } else if ( type == QUAD ) {
            const float3 O = TransformPosition( ray.O, InvertedTransform );
            const float3 D = TransformVector( ray.D, InvertedTransform );
            const float t = O.y / -D.y;
            const float size = data[0].x;
            if ( t < ray.t && t > EPS ) {
                float3 I = O + t * D;
                if ( I.x > -size && I.x < size && I.z > -size && I.z < size ) {
                    ray.t = t;
                    ray.objIdx = objIdx;
                }
            }
        } else if ( type == TRIANGLE ) {
            float3 A = TransformPosition( data[0], Transform );
            float3 B = TransformPosition( data[1], Transform );
            float3 C = TransformPosition( data[2], Transform );

            float3 AB = B - A;
            float3 AC = C - A;

            float denom = dot( cross( ray.D, AC ), AB );
            // if denom is effectively 0 there is no intersection
            if ( abs( denom ) < CL_DBL_EPSILON ) return;

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
        } else {
            printf( "Impossible primitive type given!\tintersect\n" ); // maybe we should crash here...
        }
    }

    /**
    * Get the normal of this primitive at a given intersection point
    **/
    float3 GetNormal( const float3 I ) const {
        if ( type == SPHERE ) {
            float3 center = TransformPosition( float3( 0 ), Transform );
            return ( I - center ) * data[0].z;
        } else if ( type == PLANE ) {
            return data[0];
        } else if ( type == CUBE ) {
            // transform intersection point to object space
            float3 objI = TransformPosition( I, InvertedTransform );
            // determine normal in object space
            float3 N = float3( -1, 0, 0 );
            float d0 = fabs( objI.x - data[0].x ), d1 = fabs( objI.x - data[1].x );
            float d2 = fabs( objI.y - data[0].y ), d3 = fabs( objI.y - data[1].y );
            float d4 = fabs( objI.z - data[0].z ), d5 = fabs( objI.z - data[1].z );
            float minDist = d0;
            if ( d1 < minDist ) minDist = d1, N.x = 1;
            if ( d2 < minDist ) minDist = d2, N = float3( 0, -1, 0 );
            if ( d3 < minDist ) minDist = d3, N = float3( 0, 1, 0 );
            if ( d4 < minDist ) minDist = d4, N = float3( 0, 0, -1 );
            if ( d5 < minDist ) minDist = d5, N = float3( 0, 0, 1 );
            // return normal in world space
            return TransformVector( N, Transform );
        } else if ( type == QUAD ) {
            return TransformVector( float3( 0, -1, 0 ), Transform );
        } else if ( type == TRIANGLE ) {
            float3 baseN = normalize( cross( data[2] - data[0], data[1] - data[0] ) );
            return TransformVector( baseN, Transform );
        } else {
            printf( "Impossible primitive type given!\tnormal\n" ); // maybe we should crash here...
        }
    }

    /**
    * Get the minimum coordinates needed for a bounding box containing this primitive
    **/
    float3 GetAABBMin() const {
        if ( type == SPHERE ) {
            return TransformPosition( float3( 0 ), Transform ) - float3( data[0].x );
        } else if ( type == PLANE ) {
            return -1e30f; // TODO
        } else if ( type == CUBE ) {
            float3 minPoint = TransformPosition( data[0], Transform );
            minPoint = fminf( minPoint, TransformPosition( float3( data[1].x, data[0].y, data[0].z ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( data[0].x, data[1].y, data[0].z ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( data[0].x, data[0].y, data[1].z ), Transform ) );

            minPoint = fminf( minPoint, TransformPosition( data[1], Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( data[0].x, data[1].y, data[1].z ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( data[1].x, data[0].y, data[1].z ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( data[1].x, data[1].y, data[0].z ), Transform ) );
            return minPoint;
        } else if ( type == QUAD ) {
            float size = data[0].x;
            float3 minPoint = TransformPosition( float3( -size, 0, -size ), Transform );
            minPoint = fminf( minPoint, TransformPosition( float3( -size, 0, size ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( size, 0, -size ), Transform ) );
            minPoint = fminf( minPoint, TransformPosition( float3( size, 0, size ), Transform ) );
            return minPoint;
        } else if ( type == TRIANGLE ) {
            float3 A = TransformPosition( data[0], Transform );
            float3 B = TransformPosition( data[1], Transform );
            float3 C = TransformPosition( data[2], Transform );

            return fminf( A, fminf( B, C ) );
        } else {
            printf( "Impossible primitive type given!\taabb min\n" ); // maybe we should crash here...
        }
    }

    /**
    * Get the maximum coordinates needed for a bounding box containing this primitive
    **/
    float3 GetAABBMax() const {
        if ( type == SPHERE ) {
            return TransformPosition( float3( 0 ), Transform ) + float3( data[0].x );
        } else if ( type == PLANE ) {
            return 1e30f; // TODO
        } else if ( type == CUBE ) {
            float3 maxPoint = TransformPosition( data[0], Transform );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[1].x, data[0].y, data[0].z ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[0].x, data[1].y, data[0].z ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[0].x, data[0].y, data[1].z ), Transform ) );

            maxPoint = fmaxf( maxPoint, TransformPosition( data[1], Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[0].x, data[1].y, data[1].z ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[1].x, data[0].y, data[1].z ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( data[1].x, data[1].y, data[0].z ), Transform ) );
            return maxPoint;
        } else if ( type == QUAD ) {
            float size = data[0].x;
            float3 maxPoint = TransformPosition( float3( -size, 0, -size ), Transform );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( -size, 0, size ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( size, 0, -size ), Transform ) );
            maxPoint = fmaxf( maxPoint, TransformPosition( float3( size, 0, size ), Transform ) );
            return maxPoint;
        } else if ( type == TRIANGLE ) {
            float3 A = TransformPosition( data[0], Transform );
            float3 B = TransformPosition( data[1], Transform );
            float3 C = TransformPosition( data[2], Transform );

            return fmaxf( A, fmaxf( B, C ) );
        } else {
            printf( "Impossible primitive type given!\taabb max\n" ); // maybe we should crash here...
        }
    }

    /**
    * Get a random point on this primitive
    **/
    float3 GetRandomPoint() const {
        if ( type == SPHERE ) {
            float3 point = float3( 1 );
            // uniformly distributed vector within a sphere
            while ( sqrLength( point ) > 1 ) {
                point = float3( RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f );
            }

            // scale it and return
            return TransformPosition( normalize( point ) * data[0].x, Transform );
        } else if ( type == PLANE ) {
            return -data[0] * data[1].x; // TODO
        } else if ( type == CUBE ) {
            float u = RandomFloat() * 2.0f - 1.0f;
            float v = RandomFloat() * 2.0f - 1.0f;
            float faceSign = Rand( 6 );
            float3 point = float3( 0 );
            if ( faceSign < 1 ) {
                point = float3( -1, u, v ) * data[1];
            } else if ( faceSign < 2 ) {
                point = float3( 1, u, v ) * data[1];
            } else if ( faceSign < 3 ) {
                point = float3( u, -1, v ) * data[1];
            } else if ( faceSign < 4 ) {
                point = float3( u, 1, v ) * data[1];
            } else if ( faceSign < 5 ) {
                point = float3( u, v, -1 ) * data[1];
            } else {
                point = float3( u, v, 1 ) * data[1];
            }

            return TransformPosition( point, Transform );
        } else if ( type == QUAD ) {
            float size = data[0].x;
            float3 randPoint = float3( size * ( RandomFloat() - 1.0f ), size * ( RandomFloat() - 1.0f ), 0.0f );
            return TransformPosition( randPoint, Transform );
        } else if ( type == TRIANGLE ) {
            float u = RandomFloat();
            float v = RandomFloat();
            while ( ( u + v ) > 1 ) {
                u = RandomFloat();
                v = RandomFloat();
            }

            return  TransformPosition( data[0] + u * data[1] + v * data[2], Transform );
        } else {
            printf( "Impossible primitive type given!\trandom point\n" ); // maybe we should crash here...
        }
    }

    float3 GetCentroid() const {
        return TransformPosition( centroid, Transform );
    }

    /**
    * Get the area of this primtive
    **/
    float GetArea() const {
        if ( type == SPHERE ) {
            return 4.0f * PI * data[0].y;
        } else if ( type == PLANE ) {
            return 1e30f; // TODO
        } else if ( type == CUBE ) {
            float3 s = 2.0f * data[1];
            return 2.0f * ( s.x * s.y + s.x * s.z + s.y * s.z );
        } else if ( type == QUAD ) {
            float size = 2.0f * data[0].x;
            return size * size;
        } else if ( type == TRIANGLE ) {
            float3 AB = data[1] - data[0];
            float3 AC = data[2] - data[0];
            return 0.5f * length( cross( AB, AC ) );
        } else {
            printf( "Impossible primitive type given!\tarea\n" ); // maybe we should crash here...
        }
    }

    // -----------------------------------------------------------
    // Sphere data storage: 
    // 0: x -> r    y -> r^2    z -> 1 / r
    // -----------------------------------------------------------
    static Primitive createSphere( uint& objIdx, float3 position, float r ) {
        // pack needed data into the data array of float3's
        vector<float3> sphereData;
        // save the r because we have the space, might as well.
        sphereData.push_back( float3(r, r * r, 1.0f / r) );
        // switch to using the matrix as position thing instead of p
        mat4 T = mat4::Translate( position );
        return Primitive( sphereData, SPHERE, objIdx, T );
    }

    // -----------------------------------------------------------
    // Plane data storage:
    // 0: normal
    // 1: x -> distance
    // -----------------------------------------------------------
    static Primitive createPlane( uint& objIdx, float3 normal, float distance ) {
        vector<float3> planeData;
        planeData.push_back( float3( normal ) );
        planeData.push_back( float3( distance, 0.0f, 0.0f ) );
        return Primitive( planeData, PLANE, objIdx );
    }

    // -----------------------------------------------------------
    // Cube data storage:
    // 0: -0.5 * size ( far bottom left )
    // 1:  0.5 * size ( close top right )
    // data == old b
    // -----------------------------------------------------------
    static Primitive createCube( uint& objIdx, float3 pos, float3 size, mat4 T = mat4::Identity() ) {
        mat4 transformCube = T;
        if ( length( pos ) > FLT_EPSILON ) {
            transformCube = T * mat4::Translate( pos );
        }

        vector<float3> cubeData;
        cubeData.push_back( float3( -0.5f * size ) );
        cubeData.push_back( float3(  0.5f * size ) );
        return Primitive( cubeData, CUBE, objIdx, transformCube );
    }


    // -----------------------------------------------------------
    // Quad data storage:
    // 0: x -> 0.5 * size
    // -----------------------------------------------------------
    static Primitive createQuad( uint& objIdx, float size, mat4 T = mat4::Identity() ) {
        vector<float3> quadData;
        quadData.push_back( float3( 0.5f * size, 0.0f, 0.0f ) );
        return Primitive( quadData, QUAD, objIdx, T );
    }

    static Primitive createTriangle( uint& objIdx, float3 v0, float3 v1, float3 v2 ) {
        vector<float3> triangleData;
        triangleData.push_back( float3( v0 ) );
        triangleData.push_back( float3( v1 ) );
        triangleData.push_back( float3( v2 ) );
        return Primitive( triangleData, TRIANGLE, objIdx );
    }

    // -----------------------------------------------------------
    // Miscellanious helping functions
    // -----------------------------------------------------------
    void setTextureCoordsCube( Ray& ray ) const {
        // transform intersection point to object space
        float3 objI = TransformPosition( ray.IntersectionPoint(), InvertedTransform );
        float uc, vc;

        float d0 = fabs( objI.x - data[0].x ), d1 = fabs( objI.x - data[1].x );
        float d2 = fabs( objI.y - data[0].y ), d3 = fabs( objI.y - data[1].y );
        float d4 = fabs( objI.z - data[0].z ), d5 = fabs( objI.z - data[1].z );
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
        uc = 0.5f * ( uc / data[1].x + 1.0f );
        vc = 0.5f * ( vc / data[1].x + 1.0f );

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
};