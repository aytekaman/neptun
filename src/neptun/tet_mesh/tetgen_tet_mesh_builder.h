#pragma once
#include "tetgen/tetgen.h"
#include "tet_mesh_builder.h"


namespace {
    // Naming conflicting problems :(
    void tetrahedralize2(char* str, tetgenio* in, tetgenio* out){
        tetrahedralize(str, in, out);
    }
}

// Abstract class for all of the tetmesh builders
class TetgenTetMeshBuilder : public TetMeshBuilder{
public:
    // Construct tetmesh
    int tetrahedralize(TetMeshIn& in, TetMeshOut& out) const override{
        tetgenio in_data;
        tetgenio out_data;
        convert_input(in, in_data);

        bool success = true;

        //Run tetgen
        try{
            constexpr int BUFFER_SIZE = 128;
            char str[BUFFER_SIZE];
            if (in.preserve_triangles){
                #ifdef HAVE_SNPRINTF
                    snprintf(str, BUFFER_SIZE, "q%.2fYnAfQ", in.quality);
                #else
                    sprintf_s(str, BUFFER_SIZE, "q%.2fYnAfQ", in.quality);
                #endif
            } else{
                #ifdef HAVE_SNPRINTF
                    snprintf(str, BUFFER_SIZE, "q%.2fnnAfQ", in.quality);
                #else
                    sprintf_s(str, BUFFER_SIZE, "q%.2fnnAfQ", in.quality);
                #endif
            }
            tetrahedralize2(str, &in_data,  &out_data);
        } catch (int a) {
            success = false;
            return a;
        }

        if (!success)
            return false;

        convert_output(out_data, out);
        
        return 0;
    }
private:
    // Maybe instead of copying use the pointers of existing strucure?
    void convert_input(TetMeshIn& src, tetgenio& dst) const{
        
        // Initialize dst
        dst.numberofpoints = src.num_points();
        dst.pointlist = new REAL[dst.numberofpoints * 3];
        dst.numberoffacets = src.num_facets();
        dst.facetlist = new tetgenio::facet[dst.numberoffacets];
        dst.facetmarkerlist = new int[dst.numberoffacets];

        // Fill dst
        for (size_t i = 0; i < dst.numberofpoints; i++){
            glm::vec3& p = src.points[i];
            dst.pointlist[(3 * i) + 0] = p.x;
            dst.pointlist[(3 * i) + 1] = p.y;
            dst.pointlist[(3 * i) + 2] = p.z;
        }

        std::memcpy(dst.facetmarkerlist, &src.facet_markerlist[0], sizeof(int) * src.num_facets());
        
        for (size_t i = 0; i < src.num_facets(); i++){
            const int current_facet_index = src.facet_indices[i];
            const int next_facet_index = (i == src.num_facets() - 1) ? src.facets_size() : src.facet_indices[i + 1];
            const int num_vertices = next_facet_index - current_facet_index;

            tetgenio::facet& f = dst.facetlist[i];
            f.numberofpolygons = 1;
            f.polygonlist = new tetgenio::polygon[f.numberofpolygons];

            f.numberofholes = 0;
            f.holelist = nullptr;

            tetgenio::polygon *p = &f.polygonlist[0];
            p->numberofvertices = num_vertices;
            p->vertexlist = new int[p->numberofvertices];
            
            // put vertices in reverse order
            // why reverse order? idk ...
            // TODO: Ask
            for (int i = next_facet_index - 1, j = 0; i >= current_facet_index; i--, j++){
                p->vertexlist[j] = src.facets[i];
            }
        }
    }

    void convert_output(tetgenio& src, TetMeshOut& dst) const{
        dst.points.resize(src.numberofpoints);
        dst.tets.resize(src.numberoftetrahedra);
        
        for (int i = 0; i < src.numberofpoints; i++)
        {
            dst.points[i] = glm::make_vec3(&src.pointlist[3 * i]);
        }

        dst.constrained_face_count = 0;

        for (int i = 0; i < src.numberoftetrahedra; i++)
        {
            dst.tets[i].region_id = (int)src.tetrahedronattributelist[i];

            for (int j = 0; j < 4; j++)
            {
                dst.tets[i].v[j] = src.tetrahedronlist[4 * i + j];
                dst.tets[i].n[j] = src.neighborlist[4 * i + j];
                dst.tets[i].face_idx[j] = src.trifacemarkerlist[src.tet2facelist[4 * i + j]];

                if (dst.tets[i].face_idx[j] > 0)
                    ++dst.constrained_face_count;

                if (dst.tets[i].n[j] == -1)
                    dst.air_region_id = dst.tets[i].region_id;
            }
        }
    }
};