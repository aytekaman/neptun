#pragma once

#include <vector>

#include "neptun/main/tet_mesh.h"
#include "glm/glm.hpp"

// Abstract class for all of the tetmesh builders
class TetMeshBuilder{
public:
    // Generic Tetmesh input
    //  facets stores the indices of vertices of a face
    //  facet_indices stores the index of the first vertex of each facet for facets array
    //  facet_markerlists store something (boundaries ??)
    struct TetMeshIn{
        // Input Data
        std::vector<glm::vec3> points;
        std::vector<int> facets;
        std::vector<int> facet_indices;
        std::vector<int> facet_markerlist;

        // Visibility of the face
        std::vector<bool> is_face_visible; 

        // Meshing arguments
        bool preserve_triangles;
        float quality;

        // num_points: number of vertices
        // num_facets: number of facets
        // facets_size: size of facets array (TODO: Rename?)
        TetMeshIn(int num_points, int num_facets, int facets_size) : points(num_points),
             facets(facets_size), facet_indices(num_facets), facet_markerlist(num_facets), 
             is_face_visible(facets_size, true), preserve_triangles(false), quality(5) {}

        // Utility methods
        int num_points() const {
            return points.size();
        }

        int num_facets() const{
            return facet_indices.size();
        }

        int facets_size() const{
            return facets.size();
        }
    };

    // Generic Tetmesh output
    struct TetMeshOut{
        // Output Data
        std::vector< glm::vec3 > points;
        std::vector< Tet > tets;
        int constrained_face_count;
        int air_region_id;

        // TODO: Diagnostic data ?
    };

    virtual ~TetMeshBuilder(){}

    // Construct tetmesh
    // Return 0 if success otherwise return error value ??
    virtual int tetrahedralize(TetMeshIn& in, TetMeshOut& out) const = 0;
};
