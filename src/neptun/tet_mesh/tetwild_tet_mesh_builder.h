#include "tet_mesh_builder.h"
#include "tetwild/tetwild.h"

#include <iostream>
#include <fstream>

class TetwildTetMeshBuilder : public TetMeshBuilder {
public:
    TetwildTetMeshBuilder(const TetParams& params): params(params){}
    // Construct tetmesh
    int tetrahedralize(TetMeshIn& in, TetMeshOut& out) const override{
        std::vector<glm::vec3> vertices = in.points;
        std::vector<int> triangles = in.facets;
        Eigen::MatrixXd VI(vertices.size(), 3), VO;
        Eigen::VectorXd AO;
        Eigen::MatrixXi FI(triangles.size() / 3, 3), FO, TO;

        for (int i = 0; i < vertices.size(); i++) {
            VI(i, 0) = vertices[i][0];
            VI(i, 1) = vertices[i][1];
            VI(i, 2) = vertices[i][2];
        }
        int k = 0;
        for (int i = 0; i < triangles.size() / 3; i++) {
            FI(i, 0) = triangles[k++];
            FI(i, 1) = triangles[k++];
            FI(i, 2) = triangles[k++];
        }

        tetwild::Args arg;
        arg.initial_edge_len_rel = params.ideal_edge_length;
        arg.max_num_passes = 15;
        arg.is_quiet = true;
        tetwild::tetrahedralization(VI, FI, VO, TO, AO, arg);
        out.constrained_face_count = 0;
        parse_tetwild(out.points, out.tets, out.constrained_face_count, out.air_region_id);

        return 0;
    }
private:
    TetParams params;
    void parse_tetwild(std::vector<glm::vec3> &points, std::vector<Tet> &tets, int &constrained_face_count, int &air_region_id) const{
        std::ifstream in("torus_output.txt");
        std::string s, t;

        // Read vertices
        in >> s >> t;

        int vertex_count = std::stoi(t.substr(1));

        for (int i = 0; i < vertex_count; i++) {
            double a, b, c;
            in >> a >> b >> c;
            //std::cout << a << " " << b << " " << c << std::endl;
            points.push_back({ a, b, c });
        }


        // Read tetrahedras
        in >> s >> t;
        int tetra_count = std::stoi(t.substr(1));
        tets = std::vector<Tet>(tetra_count);
        for (int i = 0; i < tetra_count; i++) {
            int a, b, c, d;
            in >> a >> b >> c >> d;
            tets[i].v[0] = a;
            tets[i].v[1] = b;
            tets[i].v[2] = c;
            tets[i].v[3] = d;

            tets[i].region_id = 0;
            //std::cout << a << " " << b << " " << c << " " << d << std::endl;
        }
        constrained_face_count = 0;
        // Read constrained faces
        //bool *flags = new bool[2880];
        in >> s >> t;
        for (int i = 0; i < tetra_count; i++) {
            for (int j = 0; j < 4; j++) {
                in >> s;
                int face_id = -1;
                if (s != "-") {
                    face_id = std::stoi(s) + 1;
                    //flags[face_id] = true;
                    constrained_face_count++;
                }
                tets[i].face_idx[3 - j] = face_id;
            }
        }
        //std::cout << "Unique faces: " << std::count(flags, flags + 2880, true) << std::endl;
        std::cout << "Constrained face count: " << constrained_face_count << std::endl;

        // Read neighbours
        in >> s;
        for (int i = 0; i < tetra_count; i++) {
            for (int j = 0; j < 4; j++) {
                in >> s;
                int n_id = -1;
                if (s != "-") {
                    n_id = std::stoi(s);
                }
                tets[i].n[j] = n_id;
            }
        }
        air_region_id = 0;

    }
};