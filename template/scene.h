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
            red = new Diffuse( float3( 0.95f, 0.05f, 0.05f ) );
            green = new Diffuse( float3( 0.05f, 0.95f, 0.05f ) );
            blue = new Diffuse( float3( 0.05f, 0.05f, 0.95f ) );
            white = new Diffuse( float3( 0.95f, 0.95f, 0.95f ) );

            mirror = new Mirror( float3( 0.9f, 0.9f, 0.9f ) );

            checkerboard = new Checkerboard( float3( 0.1f, 0.1f, 0.1f ), float3( 0.9f, 0.9f, 0.9f ), 0.85f );

            glass = new Dielectric( float3( 0.5f, 0.5f, 0.5f ), 1.52f );
            diamond = new Dielectric( float3( 0.5f, 2.5f, 0.5f ), 2.42f );

            lamp = new Light( float3( 24.0f, 24.0f, 22.0f ), float3( 0.0f, -1.0f, 0.0f )  );

            earth = new TextureMaterial( "assets/earth.png" );

            // -----------------------------------------------------------
            // Basic primitives for a simple scene
            // -----------------------------------------------------------
            primitives.push_back( Primitive::createQuad( primitiveCount, 2, lamp ) );
            primitives.push_back( Primitive::createSphere( primitiveCount, float3( 0 ), 0.5f, earth ) );
            primitives.push_back( Primitive::createCube( primitiveCount, float3( 0.0f ), float3( 1.15f ), diamond ) );
            primitives.push_back( Primitive::createQuad( primitiveCount, 50, checkerboard, mat4::Translate( float3( 0.0f, -1.0f, 0.0f ) ) ) );

            // -----------------------------------------------------------
            // mirror tetrahedron
            // -----------------------------------------------------------
            mat4 tetTransform = 
                mat4::Translate( float3( -3.0f, 0.5f, 0.5f ) ) * 
                mat4::RotateX( 1.5 * PI ) * mat4::RotateY( 0.75 * PI ) * mat4::RotateZ( 0.25 * PI ) * 
                mat4::Scale( 0.0075f );
            LoadModel( "assets/tetrahedron.obj", primitiveCount, mirror, tetTransform );

            // -----------------------------------------------------------
            // rainbow of teapots
            // -----------------------------------------------------------
            mat4 teapotTransform = mat4::Translate( float3( -3.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, new Diffuse( float3( 0.58f, 0.00f, 0.83f ) ), teapotTransform );
            teapotTransform = mat4::Translate( float3( -2.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, new Diffuse( float3( 0.29f, 0.00f, 0.51f ) ), teapotTransform );
            teapotTransform = mat4::Translate( float3( -1.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, blue, teapotTransform );
            teapotTransform = mat4::Translate( float3(  0.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, green, teapotTransform );
            teapotTransform = mat4::Translate( float3( 1.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, new Diffuse( float3( 1.0f, 1.0f, 0.0f ) ), teapotTransform);
            teapotTransform = mat4::Translate( float3( 2.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, new Diffuse( float3( 1.0f, 0.5f, 0.0f ) ), teapotTransform );
            teapotTransform = mat4::Translate( float3( 3.0f, 0.5f, 5.0f ) );
            LoadModel( "assets/teapot.obj", primitiveCount, red, teapotTransform );

            // -----------------------------------------------------------
            // lil shiba doggy - 76000 Vertices, 5000 Polygons
            // -----------------------------------------------------------
            mat4 ShibaTransform = mat4::Translate( float3( 0.0f, -0.9f, 2.0f ) ) * mat4::RotateY( PI ) * mat4::Scale( 5.0f );
            LoadModel( "assets/Shiba.obj", primitiveCount, new Dielectric( float3( 3.5f, 3.5f, 0.5f ), 1.52f ), ShibaTransform );
            SetTime( 0 );

            // -----------------------------------------------------------
            // and a stretching cat - 326641 vertices, 653278 polygons
            // -----------------------------------------------------------
            mat4 CatTransform = mat4::Translate( float3( 4.0f, 0.3f, 1.0f ) ) * mat4::RotateY( 1.5f * PI );
            LoadModel( "assets/CatStretching.obj", primitiveCount, new Dielectric( float3( 1.5f, 5.0f, 2.5f ), 1.52f ), CatTransform );
            SetTime( 0 );

            // build BVH after objects are moved with setTime
            printf("Building BVH...\n");
            primitiveIndices = (uint*) MALLOC64( primitiveCount * sizeof( uint ) );
            for ( uint i = 0; i < primitiveCount; i++ ) primitiveIndices[i] = i;

            bvhNode = (BVHNode*) MALLOC64( ( 2 * primitiveCount + 1 ) * sizeof( BVHNode ) );
            BuildBVH();
            printf( "Finished BVH!\n" );

            // this is outdated now
            // plane[0] = Plane( primitiveCount + 1, float3( 1, 0, 0 ), 3 );			// 10000: left wall
            // plane[1] = Plane( primitiveCount + 2, float3( -1, 0, 0 ), 2.99f );		// 10001: right wall
            // plane[2] = Plane( primitiveCount + 3, float3( 0, 1, 0 ), 1 );			// 10002: floor
            // plane[3] = Plane( primitiveCount + 4, float3( 0, -1, 0 ), 2 );			// 10003: ceiling
            // plane[4] = Plane( primitiveCount + 5, float3( 0, 0, 1 ), 3 );			// 10004: front wall
            // plane[5] = Plane( primitiveCount + 6, float3( 0, 0, -1 ), 3.99f );		// 10005: back wall
            // Note: once we have triangle support we should get rid of the class
            // hierarchy: virtuals reduce performance somewhat.
        }

        void LoadModel( const string fileName, uint& objIdx, ObjectMaterial* material, mat4 transform = mat4::Identity() ) {
            tinyobj::attrib_t attrib;
            vector<tinyobj::shape_t> shapes;
            vector<tinyobj::material_t> materials;
            string warn;
            string err;
            bool ret = tinyobj::LoadObj( &attrib, &shapes, &materials, &warn, &err, fileName.c_str() );

            // we get some warnings for mtls not being found, we don't use those anyway atm so ignore those
            //if ( !warn.empty() ) printf( "Model loader warning: %s\n", warn.c_str() );
            if ( !err.empty() ) printf( "Model loader error: %s\n", err.c_str() );
            if ( !ret ) {
                printf( "Failed to load/parse %s.\n", fileName.c_str() );
                return;
            }

            // Loop over shapes
            for ( size_t s = 0; s < shapes.size(); s++ ) {
                // Loop over faces(polygon)
                size_t index_offset = 0;
                for ( size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++ ) {
                    size_t fv = size_t( shapes[s].mesh.num_face_vertices[f] );

                    vector<float3> triV;
                    // Loop over vertices in the face.
                    for ( size_t v = 0; v < fv; v++ ) {
                        // access to vertex
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                        tinyobj::real_t vx = attrib.vertices[3 * size_t( idx.vertex_index ) + 0];
                        tinyobj::real_t vy = attrib.vertices[3 * size_t( idx.vertex_index ) + 1];
                        tinyobj::real_t vz = attrib.vertices[3 * size_t( idx.vertex_index ) + 2];

                        triV.push_back( TransformPosition( float3( vx, vy, vz ), transform));
                        if ( triV.size() == 3 ) {
                            primitives.push_back( Primitive::createTriangle( objIdx, triV[0], triV[1], triV[2], material ) );
                            triV.erase( triV.begin() );
                        }
                    }

                    index_offset += fv;
                }
            }

            printf( "Finished reading %s!\n", fileName.c_str() );
        }

        void BuildBVH() {
            BVHNode& root = bvhNode[rootNodeIdx];
            root.leftFirst = 0;
            root.primitiveCount = primitiveCount;
            nodesUsed = 2; // skip a node because memory alignment

            UpdateNodeBounds( rootNodeIdx );
            Subdivide( rootNodeIdx );
        }

        void Subdivide( uint nodeIdx ) {
            // terminate recursion
            BVHNode& node = bvhNode[nodeIdx];
            // determine split axis using SAH
            int axis;
            float splitPos;
            float splitCost = FindBestSplitPlane( node, axis, splitPos );
            float nosplitCost = calculateNodeCost( node );
            if ( splitCost >= nosplitCost ) return;

            // in-place partition
            int i = node.leftFirst;
            int j = i + node.primitiveCount - 1;
            while ( i <= j ) {
                if ( primitives[primitiveIndices[i]].GetCentroid()[axis] < splitPos ) {
                    i++;
                } else {
                    swap( primitiveIndices[i], primitiveIndices[j--] );
                }
            }

            // abort split if one of the sides is empty
            int leftCount = i - node.leftFirst;
            if ( leftCount == 0 || leftCount == node.primitiveCount ) return;

            // create child nodes
            int leftChildIdx = nodesUsed++;
            int rightChildIdx = nodesUsed++;
            // left child
            bvhNode[leftChildIdx].leftFirst = node.leftFirst;
            bvhNode[leftChildIdx].primitiveCount = leftCount;
            // right child
            bvhNode[rightChildIdx].leftFirst = i;
            bvhNode[rightChildIdx].primitiveCount = node.primitiveCount - leftCount;
            // current node becomes a non-leaf
            node.leftFirst = leftChildIdx;
            node.primitiveCount = 0;
            // update bounds and subdivide children
            UpdateNodeBounds( leftChildIdx );
            UpdateNodeBounds( rightChildIdx );
            Subdivide( leftChildIdx );
            Subdivide( rightChildIdx );
        }

        float FindBestSplitPlane( BVHNode& node, int& axis, float& splitPos ) {
            float bestCost = 1e30f;
            for ( int a = 0; a < 3; a++ ) {
                float boundsMin = 1e30f;
                float boundsMax = -1e30f;
                for ( uint i = 0; i < node.primitiveCount; i++ ) {
                    int objIdx = primitiveIndices[node.leftFirst + i];
                    Primitive primitive = primitives[objIdx];
                    boundsMin = min( boundsMin, primitive.GetCentroid()[a] );
                    boundsMax = max( boundsMax, primitive.GetCentroid()[a] );
                }

                if ( boundsMin == boundsMax ) continue;

                // populate the bins
                BVHBin bin[BIN_COUNT];
                float scale = BIN_COUNT / ( boundsMax - boundsMin );
                for ( uint i = 0; i < node.primitiveCount; i++ ) {
                    int objIdx = primitiveIndices[node.leftFirst + i];
                    Primitive primitive = primitives[objIdx];
                    int binIdx = min( BIN_COUNT - 1, (int) ( ( primitive.GetCentroid()[a] - boundsMin ) * scale ) );

                    bin[binIdx].primitiveCount++;
                    bin[binIdx].bounds.Grow( primitive.GetAABBMin() );
                    bin[binIdx].bounds.Grow( primitive.GetAABBMax() );
                }

                // gather data for the 7 planes between the 8 bins
                float leftArea[BIN_COUNT - 1];
                float rightArea[BIN_COUNT - 1];
                int leftCount[BIN_COUNT - 1];
                int rightCount[BIN_COUNT - 1];
                aabb leftBox;
                aabb rightBox;
                int leftSum = 0;
                int rightSum = 0;
                for ( int i = 0; i < BIN_COUNT - 1; i++ ) {
                    leftSum += bin[i].primitiveCount;
                    leftCount[i] = leftSum;
                    leftBox.Grow( bin[i].bounds );
                    leftArea[i] = leftBox.Area();

                    rightSum += bin[BIN_COUNT - 1 - i].primitiveCount;
                    rightCount[BIN_COUNT - 2 - i] = rightSum;
                    rightBox.Grow( bin[BIN_COUNT - 1 - i].bounds );
                    rightArea[BIN_COUNT - 2 - i] = rightBox.Area();
                }

                // calculate SAH cost for the 7 planes
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
            for ( uint first = node.leftFirst, i = 0; i < node.primitiveCount; i++ ) {
                uint objIdx = primitiveIndices[first + i];
                Primitive primitive = primitives[objIdx];
                node.aabbMin = fminf( node.aabbMin, primitive.GetAABBMin() );
                node.aabbMax = fmaxf( node.aabbMax, primitive.GetAABBMax() );
            }
        }

        ObjectMaterial* GetMaterial( int objIdx ) {
            return primitives[objIdx].material;
        }

        void SetTime( float t ) {
            // default time for the scene is simply 0. Updating/ the time per frame 
            // enables animation. Updating it per ray can be used for motion blur.
            animTime = t;
            // light source animation: swing
            mat4 M1base = mat4::Translate( float3( 0, 2.6f, 2 ) );
            mat4 M1 = M1base * mat4::RotateZ( sinf( animTime * 0.6f ) * 0.1f ) * mat4::Translate( float3( 0, -0.9, 0 ) );
            primitives[0].UpdateTransform(  M1 );
            // sphere animation: bounce
            float tm = 1 - sqrf( fmodf( animTime, 2.0f ) - 1 );
            primitives[1].UpdateTransform( mat4::Translate( float3( -1.4f, -0.5f + tm, 2 ) ) );
            // cube animation: spin
            mat4 M2base = mat4::RotateX( PI / 4 ) * mat4::RotateZ( PI / 4 );
            mat4 M2 = mat4::Translate( float3( 1.4f, 0, 2 ) ) * mat4::RotateY( animTime * 0.5f ) * M2base;
            primitives[2].UpdateTransform( M2 );
        }
        int GetRandomLight() const {
            return 0; // only one light atm...
        }
        float3 GetLightColor( int objIdx, Ray& ray ) {
            return GetMaterial( objIdx )->GetColor( ray );
        }
        float GetArea( int objIdx ) const {
            return primitives[objIdx].GetArea();
        }
        float3 GetLightPos( int objIdx ) const {
            return primitives[objIdx].GetRandomPoint();
        }
        float3 GetLightColor() const {
            return float3( 24, 24, 22 );
        }
        float3 GetLightDir() const {
            return float3( 0.0f, -1.0f, 0.0f );
        }

        void IntersectBVH( Ray& ray ) {
            BVHNode* node = &bvhNode[rootNodeIdx];
            BVHNode* stack[64];
            uint stackPtr = 0;
            while ( true ) {
                if ( node->isLeaf() ) {
                    for ( uint i = 0; i < node->primitiveCount; i++ ) {
                        int objIdx = primitiveIndices[node->leftFirst + i];
                        primitives[objIdx].Intersect( ray );
                    }

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
                    node = stack[--stackPtr];
                } else {
                    node = child1;
                    if ( dist2 != 1e30f ) stack[stackPtr++] = child2;
                }
            }
        }

        void IntersectBVHPacket( RayPacket& rays ) {
            rays.firstActive = 0;
            // start at the root
            BVHNode* node = &bvhNode[rootNodeIdx];

            // stack stuff
            uint stackPtr = 0;
            BVHNode* stack[64];
            while ( true ) {
                Ray testRay = rays.GetRay( rays.firstActive );

                if ( HitsAABB( testRay, node->aabbMin, node->aabbMax ) ) {
                    if ( node->isLeaf() ) {
                        for ( int i = 0; i < PACKET_SIZE; i++ ) {
                            Ray ray = rays.GetRay( i );

                            for ( uint idx = 0; idx < node->primitiveCount; idx++ ) {
                                int objIdx = primitiveIndices[node->leftFirst + idx];
                                primitives[objIdx].Intersect( ray );
                            }

                            if ( ray.t < rays.t[i] ) {
                                rays.t[i] = ray.t;
                                rays.u[i] = ray.u;
                                rays.v[i] = ray.v;
                                rays.objIdx[i] = ray.objIdx;
                            }
                        }

                        // stop the loop if we are at stack pointer 0, aka the root
                        if ( stackPtr == 0 ) break;

                        node = stack[--stackPtr];
                        continue;
                    }
                } else {
                    // go over all rays to find the first one that does intersect with this node
                    bool foundHit = false;
                    for ( int i = 0; i < PACKET_SIZE; i++ ) {
                        testRay = rays.GetRay( i );

                        if ( HitsAABB( testRay, node->aabbMin, node->aabbMax ) ) {
                            rays.firstActive = i;
                            foundHit = true;
                            break;
                        }
                    }

                    if ( !foundHit ) {
                        if ( stackPtr == 0 ) break;

                        node = stack[--stackPtr];
                        continue;
                    }

                    if ( node->isLeaf() ) {
                        for ( int i = rays.firstActive; i < PACKET_SIZE; i++ ) {
                            Ray ray = rays.GetRay( i );

                            for ( uint idx = 0; idx < node->primitiveCount; idx++ ) {
                                int objIdx = primitiveIndices[node->leftFirst + idx];
                                primitives[objIdx].Intersect( ray );
                            }

                            if ( ray.t < rays.t[i] ) {
                                rays.t[i] = ray.t;
                                rays.u[i] = ray.u;
                                rays.v[i] = ray.v;
                                rays.objIdx[i] = ray.objIdx;
                            }
                        }

                        if ( stackPtr == 0 ) break;

                        node = stack[--stackPtr];
                        continue;
                    }
                }

                BVHNode* child1 = &bvhNode[node->leftFirst];
                BVHNode* child2 = &bvhNode[node->leftFirst + 1];
                float dist1 = IntersectAABB( testRay, child1->aabbMin, child1->aabbMax );
                float dist2 = IntersectAABB( testRay, child2->aabbMin, child2->aabbMax );
                if ( dist1 > dist2 ) {
                    swap( dist1, dist2 );
                    swap( child1, child2 );
                }

                node = child1;
                stack[stackPtr++] = child2;
            }
        }

        bool HitsAABB( const Ray& ray, const float3 bmin, const float3 bmax ) {
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
            return tmax >= tmin && tmin < ray.t&& tmax > 0;
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

        bool IsOccluded( Ray& ray ) {
            BVHNode* node = &bvhNode[rootNodeIdx], * stack[64];
            uint stackPtr = 0;
            bool isOccluded = false;
            while ( true ) {
                if ( node->isLeaf() ) {
                    for ( uint i = 0; i < node->primitiveCount; i++ ) {
                        int objIdx = primitiveIndices[node->leftFirst + i];
                        if ( primitives[objIdx].Hit( ray ) ) return true;
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
            float3 N = primitives[ objIdx ].GetNormal( I );
            if ( dot( N, wo ) > 0 ) N = -N; // hit backside / inside
            return N;
        }
        __declspec( align( 64 ) ) // start a new cacheline here
            float animTime = 0;

        vector<Primitive> primitives;
        uint* primitiveIndices;

        ObjectMaterial* red;
        ObjectMaterial* green;
        ObjectMaterial* blue;
        ObjectMaterial* white;

        ObjectMaterial* mirror;

        ObjectMaterial* checkerboard;

        ObjectMaterial* glass;
        ObjectMaterial* diamond;

        ObjectMaterial* lamp;

        ObjectMaterial* earth;

        BVHNode* bvhNode;
        uint rootNodeIdx = 0;
        uint nodesUsed;
        uint primitiveCount = 0;
    };

}