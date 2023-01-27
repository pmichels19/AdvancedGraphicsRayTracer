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

            checkerboard = new Checkerboard( float3( 0.1f, 0.1f, 0.1f ), float3( 0.9f, 0.9f, 0.9f ) );

            glass = new Dielectric( float3( 0.5f, 0.5f, 0.5f ), 1.52f );
            diamond = new Dielectric( float3( 0.5f, 2.5f, 0.5f ), 2.42f );

            lamp = new Light( float3( 24.0f, 24.0f, 22.0f ), float3( 0.0f, -1.0f, 0.0f )  );

            earth = new TextureMaterial( "assets/earth.png" );

            // -----------------------------------------------------------
            // Following two triangles show the difference in SBVH and BVH very clearly
            // -----------------------------------------------------------
            primitives.push_back( Primitive::createTriangle( primitiveCount, float3( -5, -2, 3 ), float3( -3, -4, 3 ), float3( 3, 4, 3 ), red ) );
            primitives.push_back( Primitive::createTriangle( primitiveCount, float3( 5, 2, 3 ), float3( -3, -4, 3 ), float3( 3, 4, 3 ), blue ) );

            // -----------------------------------------------------------
            // BVH Visualization - 2 plane scene - ~500K triangles
            // -----------------------------------------------------------
            //float planeSize = 5.0f;
            //mat4 planeScale = mat4::Scale( planeSize / 2822.548584f );
            //mat4 planeTransform = mat4::Translate( float3( 0.5f + planeSize, 0.0f, 3.0f ) ) * planeScale;
            //LoadModel( "assets/airways.obj", primitiveCount, blue, planeTransform );
            
            //planeTransform = mat4::Translate( float3( -0.5f, 0.0f, 3.0f ) ) * mat4::RotateX( -0.5f * PI ) * planeScale;
            //LoadModel( "assets/airways.obj", primitiveCount, red, planeTransform );

            // -----------------------------------------------------------
            // A scene with a big plane - 161K triangles
            // -----------------------------------------------------------
            //mat4 airplaneTransform = mat4::Translate( float3( 1.0f, 1.0f, 5.0f ) ) * mat4::RotateZ( 0.1f * PI ) * mat4::RotateX( -0.6f * PI ) * mat4::Scale( 0.005f );
            //LoadTexturedModel( "assets/airplane/airplane.obj", primitiveCount, airplaneTransform );

            // build BVH after objects are moved with setTime
            printf("Building BVH...\n");
#ifdef SPATIAL_SPLITS
            vector<uint> indices;
            for ( uint i = 0; i < primitiveCount; i++ ) indices.push_back( i );

            splits = 0;
            primitiveMap[rootNodeIdx] = indices;
            // no real prediction on how many nodes there will be, so just reserve a lot of space
            bvhNode = (BVHNode*) MALLOC64( ( 4 * primitiveCount ) * sizeof( BVHNode ) );
            BuildSBVH();
            BuildPrimitiveIndices();
#else
            primitiveIndices = (uint*) MALLOC64( primitiveCount * sizeof( uint ) );
            for ( uint i = 0; i < primitiveCount; i++ ) primitiveIndices[i] = i;

            bvhNode = (BVHNode*) MALLOC64( ( 2 * primitiveCount + 1 ) * sizeof( BVHNode ) );
            BuildBVH();
#endif
            printf( "Finished BVH!\n" );

            printf( "%d nodes for %d primitives.\n", nodesUsed - 1, primitiveCount );
            printf( "Found SAH cost of %f\n", SAHCost() );
            bvhDepth = maxDepthBVH( rootNodeIdx );
            printf( "Tree depth: %d\n", bvhDepth );
            printf( "Overlapping area: %f\n", overlap );
#ifdef SPATIAL_SPLITS
            printf( "spatial splits: %d\n", splits );
#endif
        }

        float SAHCost() {
            aabb nodeBox( bvhNode[rootNodeIdx].aabbMin, bvhNode[rootNodeIdx].aabbMax );
            float cost = nodeBox.Area() * bvhNode[rootNodeIdx].primitiveCount;
            for ( int i = 2; i < nodesUsed; i++ ) {
                BVHNode& node = bvhNode[i];
                if ( !node.isLeaf() ) continue;

                nodeBox = aabb( node.aabbMin, node.aabbMax );
                cost += nodeBox.Area() * node.primitiveCount;
            }

            return cost;
        }

        int maxDepthBVH( uint nodeIdx ) {
            BVHNode& node = bvhNode[nodeIdx];
            if ( node.isLeaf() ) {
                if ( nodeIdx == rootNodeIdx ) return 1;
                else return 0;
            }

            int lDepth = maxDepthBVH( node.leftFirst );
            int rDepth = maxDepthBVH( node.leftFirst + 1 );
            return max( lDepth, rDepth ) + 1;
        }

        void LoadTexturedModel( const string fileName, uint& objIdx, mat4 transform = mat4::Identity() ) {
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

            int base_id = modelMaterials.size();
            for ( tinyobj::material_t mat : materials ) {
                modelMaterials.push_back( new TextureMaterial( mat.diffuse_texname.c_str() ) );
            }

            // Loop over shapes
            for ( size_t s = 0; s < shapes.size(); s++ ) {
                // Loop over faces(polygon)
                size_t index_offset = 0;
                for ( size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++ ) {
                    size_t fv = size_t( shapes[s].mesh.num_face_vertices[f] );

                    vector<float3> triV;
                    vector<float2> triVt;
                    int material_id = base_id + shapes[s].mesh.material_ids[f];
                    // Loop over vertices in the face.
                    for ( size_t v = 0; v < fv; v++ ) {
                        // access to vertex
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                        tinyobj::real_t vx = attrib.vertices[3 * size_t( idx.vertex_index ) + 0];
                        tinyobj::real_t vy = attrib.vertices[3 * size_t( idx.vertex_index ) + 1];
                        tinyobj::real_t vz = attrib.vertices[3 * size_t( idx.vertex_index ) + 2];

                        // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                        if ( idx.texcoord_index >= 0 ) {
                            tinyobj::real_t tx = attrib.texcoords[2 * size_t( idx.texcoord_index ) + 0];
                            tinyobj::real_t ty = attrib.texcoords[2 * size_t( idx.texcoord_index ) + 1];
                            triVt.push_back( float2( tx, 1.0f - ty ) );
                        }

                        triV.push_back( TransformPosition( float3( vx, vy, vz ), transform ) );
                        if ( triV.size() == 3 ) {
                            primitives.push_back( Primitive::createTexturedTriangle( objIdx, triV[0], triV[1], triV[2], triVt[0], triVt[1], triVt[2], modelMaterials[material_id] ));
                            triV.erase( triV.begin() );
                            triVt.erase( triVt.begin() );
                        }
                    }

                    index_offset += fv;
                }
            }

            printf( "Finished reading %s!\n", fileName.c_str() );
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

                        triV.push_back( TransformPosition( float3( vx, vy, vz ), transform ) );
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

        float calculateNodeCost( BVHNode& node ) {
            float3 e = node.aabbMax - node.aabbMin;
            float area = e.x * e.y + e.y * e.z + e.z * e.x;
            return node.primitiveCount * area;
        }

        ObjectMaterial* GetMaterial( int objIdx ) {
            return primitives[objIdx].material;
        }

        void SetTime( float t ) {
            // default time for the scene is simply 0. Updating/ the time per frame 
            // enables animation. Updating it per ray can be used for motion blur.
            animTime = t;
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

        float3 BVHVisualization( Ray& ray ) {
            BVHNode* node = &bvhNode[rootNodeIdx];
            BVHNode* stack[64];
            uint stackPtr = 0;
            int nodeTraversals = 0;
            int intersections = 0;
            while ( true ) {
                nodeTraversals++;
                if ( node->isLeaf() ) {
                    intersections++;

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

            //return float3( 0.0f, 0.25f * nodeTraversals / (float) bvhDepth, 0.0f ); // bvh depth visualization
            //return float3( intersections / (float) bvhDepth, 0.0f, 0.0f ); // number of intersections visualization
            return float3( intersections / (float) bvhDepth, 0.25f * nodeTraversals / (float) bvhDepth, 0.0f ); // both
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

                            for ( uint i = 0; i < node->primitiveCount; i++ ) {
                                int objIdx = primitiveIndices[node->leftFirst + i];
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
                            for ( uint i = 0; i < node->primitiveCount; i++ ) {
                                int objIdx = primitiveIndices[node->leftFirst + i];
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
                bool hits1 = HitsAABB( ray, child1->aabbMin, child1->aabbMax );
                bool hits2 = HitsAABB( ray, child2->aabbMin, child2->aabbMax );

                if ( hits1 && hits2 ) {
                    node = child1;
                    stack[stackPtr++] = child2;
                } else if ( !( hits1 || hits2 ) ) {
                    if ( stackPtr == 0 ) break;
                    else node = stack[--stackPtr];
                } else if ( hits1 ) {
                    node = child1;
                } else {
                    node = child2;
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

        // The aabb's standard intersection method doesn't check if the two boxes actually intersect, so we do that manually
        float CalculateAABBIntersectionArea( aabb box1, aabb box2 ) {
            // minimum points
            float3 min1 = box1.bmin3;
            float3 min2 = box2.bmin3;
            // maximum points
            float3 max1 = box1.bmax3;
            float3 max2 = box2.bmax3;
            // check for overlaps
            bool xOverlap = ( min1.x <= min2.x && min2.x <= max1.x ) || ( min1.x <= max2.x && max2.x <= max1.x ) || ( min2.x <= min1.x && min1.x <= max2.x ) || ( min2.x <= max1.x && max1.x <= max2.x );
            bool yOverlap = ( min1.y <= min2.y && min2.y <= max1.y ) || ( min1.y <= max2.y && max2.y <= max1.y ) || ( min2.y <= min1.y && min1.y <= max2.y ) || ( min2.y <= max1.y && max1.y <= max2.y );
            bool zOverlap = ( min1.z <= min2.z && min2.z <= max1.z ) || ( min1.z <= max2.z && max2.z <= max1.z ) || ( min2.z <= min1.z && min1.z <= max2.z ) || ( min2.z <= max1.z && max1.z <= max2.z );
            // if no overlap, return false
            if ( !xOverlap || !yOverlap || !zOverlap ) return 0.0f;

            return box1.Intersection( box2 ).Area();
        }

#ifdef SPATIAL_SPLITS
        // -----------------------------------------------------------
        // SBVH functions
        // -----------------------------------------------------------
        void BuildSBVH() {
            BVHNode& root = bvhNode[rootNodeIdx];
            root.leftFirst = rootNodeIdx;
            root.primitiveCount = primitiveCount;
            nodesUsed = 2; // skip a node because memory alignment

            root.aabbMin = float3( 1e30f );
            root.aabbMax = float3( -1e30f );

            for ( uint objIdx : primitiveMap[rootNodeIdx] ) {
                Primitive primitive = primitives[objIdx];
                root.aabbMin = fminf( root.aabbMin, primitive.GetAABBMin() );
                root.aabbMax = fmaxf( root.aabbMax, primitive.GetAABBMax() );
            }

            Subdivide( rootNodeIdx );
        }

        void BuildPrimitiveIndices() {
            // number of references to primitives
            int references = 0;
            for ( auto iterator = primitiveMap.begin(); iterator != primitiveMap.end(); iterator++ ) {
                references += iterator->second.size();
            }

            // measure the percentage of references to primtives
            float primitiveReferencesPerc = 100.0f * ( (float) references ) / ( (float) primitiveCount );
            printf( "References to primitives: %f\n", primitiveReferencesPerc );

            int insertPointer = 0;
            primitiveIndices = (uint*) MALLOC64( references * sizeof( uint ) );
            for ( auto iterator = primitiveMap.begin(); iterator != primitiveMap.end(); iterator++ ) {
                BVHNode& node = bvhNode[iterator->first];

                node.leftFirst = insertPointer;
                for (uint objIdx : iterator->second) {
                    primitiveIndices[insertPointer] = objIdx;
                    insertPointer++;
                }
            }

            primitiveMap.clear();
        }

        void Subdivide( uint nodeIdx ) {
            BVHNode& node = bvhNode[nodeIdx];

            BVHSplit split;
            float splitCost = FindBestSplitPlane( node, split );
            float nosplitCost = calculateNodeCost( node );
            if ( splitCost >= nosplitCost ) return;

            float leftCount = split.leftChildren.size();
            float rightCount = split.rightChildren.size();
            // abort split if one of the sides is empty
            if ( leftCount == 0 || rightCount == 0 ) return;

            // update the total overlapping area
            overlap += CalculateAABBIntersectionArea( split.left, split.right );
            // update the number of spatial splits
            splits += split.splitPrimitives().size();

            // create child nodes
            int leftChildIdx = nodesUsed++;
            int rightChildIdx = nodesUsed++;
            // left child
            bvhNode[leftChildIdx].leftFirst = leftChildIdx;
            bvhNode[leftChildIdx].primitiveCount = leftCount;
            // right child
            bvhNode[rightChildIdx].leftFirst = rightChildIdx;
            bvhNode[rightChildIdx].primitiveCount = rightCount;
            // current node becomes a non-leaf
            node.leftFirst = leftChildIdx;
            node.primitiveCount = 0;
            primitiveMap.erase(nodeIdx);
            // update bounds
            bvhNode[leftChildIdx].aabbMax = float3( split.left.bmax3 );
            bvhNode[leftChildIdx].aabbMin = float3( split.left.bmin3 );
            bvhNode[rightChildIdx].aabbMax = float3( split.right.bmax3 );
            bvhNode[rightChildIdx].aabbMin = float3( split.right.bmin3 );
            // set primitives
            primitiveMap.insert( pair<uint, vector<uint>>( leftChildIdx, vector<uint>( split.leftChildren ) ) );
            primitiveMap.insert( pair<uint, vector<uint>>( rightChildIdx, vector<uint>( split.rightChildren ) ) );

            Subdivide( leftChildIdx );
            Subdivide( rightChildIdx );
        }

        float FindBestSplitPlane( BVHNode& node, BVHSplit& split ) {
            float splitCost = findBestObjectSplit( node, split );
            // area of the root
            BVHNode root = bvhNode[rootNodeIdx];
            aabb rootAABB( root.aabbMin, root.aabbMax );
            float rootArea = rootAABB.Area();
            // overlapping area of the split
            float intersectionArea = CalculateAABBIntersectionArea( split.left, split.right );
            // check if we want to do a spatial split attempt
            if ( ( intersectionArea / rootArea ) > SPATIAL_SPLIT_ALPHA ) {
                findBestSpatialSplit( node, split, splitCost );
            }

            return splitCost;
        }

        /**
        * Method for finding the best split on objects
        * This is basically the normal BVH FindBestSplitPlane with modifications to take the tight AABBs from the spatial splits into account.
        **/
        float findBestObjectSplit( BVHNode& node, BVHSplit& split ) {
            int axis = 0;

            float splitPos = 1e30f;
            float bestCost = 1e30f;

            aabb nodeBox( node.aabbMin, node.aabbMax );
            for ( int a = 0; a < 3; a++ ) {
                float boundsMin = 1e30f;
                float boundsMax = -1e30f;
                for ( int objIdx : primitiveMap[node.leftFirst] ) {
                    Primitive primitive = primitives[objIdx];
                    boundsMin = min( boundsMin, primitive.GetCentroid()[a] );
                    boundsMax = max( boundsMax, primitive.GetCentroid()[a] );
                }

                if ( boundsMin == boundsMax ) continue;

                // populate the bins
                BVHBin bin[BIN_COUNT];
                float scale = BIN_COUNT / ( boundsMax - boundsMin );
                for ( int objIdx : primitiveMap[node.leftFirst] ) {
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
                    leftBox = leftBox.Intersection( nodeBox );
                    leftArea[i] = leftBox.Area();

                    rightSum += bin[BIN_COUNT - 1 - i].primitiveCount;
                    rightCount[BIN_COUNT - 2 - i] = rightSum;
                    rightBox.Grow( bin[BIN_COUNT - 1 - i].bounds );
                    rightBox = rightBox.Intersection( nodeBox );
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

            // make the split
            for ( int objIdx : primitiveMap[node.leftFirst] ) {
                Primitive primitive = primitives[objIdx];

                if ( primitive.GetCentroid()[axis] < splitPos ) {
                    split.leftChildren.push_back( objIdx );
                    split.left.Grow( primitive.GetAABBMin() );
                    split.left.Grow( primitive.GetAABBMax() );
                } else {
                    split.rightChildren.push_back( objIdx );
                    split.right.Grow( primitive.GetAABBMin() );
                    split.right.Grow( primitive.GetAABBMax() );
                }
            }

            // clip the resulting boxes against the parent node
            split.left = split.left.Intersection( nodeBox );
            split.right = split.right.Intersection( nodeBox );

            return bestCost;
        }

        /**
        * Method for finding the best spatial split
        **/
        float findBestSpatialSplit( BVHNode& node, BVHSplit& split, float objectSplitCost ) {
            int axis = 0;
            float bestCost = objectSplitCost;
            for ( int a = 0; a < 3; a++ ) {
                float boundsMin = node.aabbMin[a];
                float boundsMax = node.aabbMax[a];

                if ( boundsMin == boundsMax ) continue;

                // do chopped binning
                BVHBin bin[BIN_COUNT];
                float scale = ( boundsMax - boundsMin ) / (float) BIN_COUNT;
                set<int> binPrimitives[BIN_COUNT];
                for ( int binIdx = 0; binIdx < BIN_COUNT; binIdx++ ) {
                    for ( int objIdx : primitiveMap[node.leftFirst] ) {
                        Primitive primitive = primitives[objIdx];

                        float bMin = boundsMin + binIdx * scale;
                        // check if the current primitive was already (partly) in the bin to the left
                        bool inPreviousBin = binIdx > 0;
                        if ( inPreviousBin ) inPreviousBin = inPreviousBin && ( binPrimitives[binIdx - 1].count( objIdx ) != 0 );

                        if ( primitive.isInBin( bMin, bMin + scale, a ) ) {
                            binPrimitives[binIdx].insert(objIdx);
                            bin[binIdx].bounds.Grow( primitive.fitInBin( bMin, bMin + scale, a) );

                            if ( !inPreviousBin ) bin[binIdx].entry++;
                        } else if ( inPreviousBin ) {
                            bin[binIdx - 1].exit++;
                        }
                    }

                    if ( binIdx == BIN_COUNT - 1 ) {
                        bin[binIdx].exit += binPrimitives[binIdx].size();
                    }
                }

                // gather data for the planes between the bins
                aabb leftBoxes[BIN_COUNT - 1];
                aabb rightBoxes[BIN_COUNT - 1];
                set<uint> leftPrimitives[BIN_COUNT - 1];
                set<uint> rightPrimitives[BIN_COUNT - 1];
                for ( int i = 0; i < BIN_COUNT - 1; i++ ) {
                    for ( int l = 0; l <= i; l++ ) {
                        leftBoxes[i].Grow( bin[l].bounds );
                        leftPrimitives[i].insert( binPrimitives[l].begin(), binPrimitives[l].end() );
                    }

                    for ( int r = i + 1; r < BIN_COUNT; r++ ) {
                        rightBoxes[i].Grow( bin[r].bounds );
                        rightPrimitives[i].insert( binPrimitives[r].begin(), binPrimitives[r].end() );
                    }
                }

                // calculate SAH cost for the planes
                scale = ( boundsMax - boundsMin ) / BIN_COUNT;
                for ( int i = 0; i < BIN_COUNT - 1; i++ ) {
                    float planeCost = leftPrimitives[i].size() * leftBoxes[i].Area() + rightPrimitives[i].size() * rightBoxes[i].Area();

                    if ( planeCost < bestCost ) {
                        split.left = aabb( leftBoxes[i] );
                        split.right = aabb( rightBoxes[i] );
                        split.leftChildren = vector<uint>( leftPrimitives[i].begin(), leftPrimitives[i].end() );
                        split.rightChildren = vector<uint>( rightPrimitives[i].begin(), rightPrimitives[i].end() );
                        axis = a;
                        bestCost = planeCost;
                    }
                }
            }

#ifdef SBVH_UNSPLITTING
            // try unsplitting some of the split primitives
            if ( split.leftChildren.size() > 0 && split.rightChildren.size() > 0 ) {
                float unsplitCost = unsplit( node, split, axis, bestCost );

                if ( unsplitCost < bestCost ) bestCost = unsplitCost;
            }
#endif

            return bestCost;
        }

        float unsplit( BVHNode& node, BVHSplit& split, int axis, float bestCost ) {
            aabb nodeBox( node.aabbMin, node.aabbMax );
            // get a list of all the split primitives
            vector<uint> intersection = split.splitPrimitives();
            // get the cost of the current, non-unsplit split
            float currentCost = bestCost;
            int lCount = split.leftChildren.size();
            int rCount = split.rightChildren.size();
            // loop over all the split objects
            for ( uint objIdx : intersection ) {
                // get the info on the primitive
                Primitive primitive = primitives[objIdx];
                aabb primBox( primitive.GetAABBMin(), primitive.GetAABBMax() );
                primBox = primBox.Intersection( nodeBox );

                // get the left and right box extended with the split primitive
                aabb lExtended = aabb::Union( split.left,  primBox );
                aabb rExtended = aabb::Union( split.right, primBox );

                // calculate the conservative estimates for unsplitting the primitive
                float lCost = lExtended.Area() * lCount + split.right.Area() * ( rCount - 1 );
                float rCost = split.left.Area() * ( lCount - 1 ) + rExtended.Area() * rCount;

                // otherwise update the cost and adjust the appropriate boxes and children of the split
                if ( lCost <= currentCost && lCost < rCost ) {
                    currentCost = lCost;
                    rCount--;
                    split.left = aabb( lExtended );
                    split.rightChildren.erase( find( split.rightChildren.begin(), split.rightChildren.end(), objIdx ) );
                } else if ( rCost <= currentCost && rCost < lCost ) {
                    currentCost = rCost;
                    lCount--;
                    split.right = aabb( rExtended );
                    split.leftChildren.erase( find( split.leftChildren.begin(), split.leftChildren.end(), objIdx ) );
                }
            }

            return currentCost;
        }
#else
        // -----------------------------------------------------------
        // Normal BVH functions
        // -----------------------------------------------------------
        void BuildBVH() {
            BVHNode& root = bvhNode[rootNodeIdx];
            root.leftFirst = 0;
            root.primitiveCount = primitiveCount;
            nodesUsed = 2; // skip a node because memory alignment

            UpdateNodeBounds( rootNodeIdx );
            Subdivide( rootNodeIdx );
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
            // update bounds
            UpdateNodeBounds( leftChildIdx );
            UpdateNodeBounds( rightChildIdx );
            // update the total overlapping area
            overlap += CalculateAABBIntersectionArea( aabb( bvhNode[leftChildIdx].aabbMin, bvhNode[leftChildIdx].aabbMax ), aabb( bvhNode[rightChildIdx].aabbMin, bvhNode[rightChildIdx].aabbMax ) );
            // subdivide children
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
#endif


        __declspec( align( 64 ) ) // start a new cacheline here
            float animTime = 0;

        vector<Primitive> primitives;
        uint* primitiveIndices;

#ifdef SPATIAL_SPLITS
        unordered_map<uint, vector<uint>> primitiveMap;
        int splits;
#endif

        int bvhDepth = 1;
        float overlap = 0.0f;

        vector<TextureMaterial*> modelMaterials;

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