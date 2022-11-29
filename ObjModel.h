#pragma once

class ObjModel {
private:
    vector<float3> vertices;
    vector<Triangle> triangles;
public:
    static int objIdxCounter;

    ObjModel( const string fileName ) {
        string content;
        ifstream fin(fileName);

        if ( !fin ) {
            printf("Couldn't open .obj file %s\n", fileName);
            return;
        }

        while ( fin >> content ) {
            switch ( *content.c_str() ) {
                case 'v':
                    float3 vertex;
                    fin >> vertex.x >> vertex.y >> vertex.z;
                    vertices.push_back( vertex );
                    break;
                case 'f':
                    int v1, v2, v3;
                    fin >> v1 >> v2 >> v3;
                    triangles.push_back( Triangle( objIdxCounter, vertices[v1], vertices[v2], vertices[v3] ));
                    objIdxCounter++;
                    break;
            }
        }

        // no need to keep this memory occupied, free it up
        vertices.clear();
    }

    void Intersect( Ray& ray ) {
        // intersect all triangles in this object
        for ( Triangle t : triangles ) {
            t.Intersect( ray );
        }
    }

    float3 GetNormal( float3 I ) {
        // TODO...
    }
};