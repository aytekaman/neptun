#include "tet_mesh_builder.h"
#include "floattetwild\FloatTetwild.h"

#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <iostream>
#include <fstream>

#include <unordered_set>

class TetwildTetMeshBuilder : public TetMeshBuilder {
public:
    TetwildTetMeshBuilder(const TetParams& params): params(params){}
    // Construct tetmesh
    int tetrahedralize(TetMeshIn& in, TetMeshOut& out) const override{
        GEO::initialize();

        std::vector<glm::vec3> vertices = in.points;
        std::vector<int> triangles = in.facets;
        Eigen::MatrixXd VI(vertices.size(), 3), VO;
        Eigen::VectorXd AO;
        Eigen::MatrixXi FI(triangles.size() / 3, 3), FO, TO;
        GEO::Mesh sf_mesh;
        GEO::CmdLine::import_arg_group("standard");
        GEO::CmdLine::import_arg_group("pre");
        GEO::CmdLine::import_arg_group("algo");

        auto &mesh_vertices = sf_mesh.vertices;
        auto &mesh_faces = sf_mesh.facets;
        for (int i = 0; i < vertices.size(); i++) {
            /*VI(i, 0) = vertices[i][0];
            VI(i, 1) = vertices[i][1];
            VI(i, 2) = vertices[i][2];*/
            double *vs = new double[3]; // TODO: Avoid memory-leak
            vs[0] = vertices[i][0];
            vs[1] = vertices[i][1];
            vs[2] = vertices[i][2];
            mesh_vertices.create_vertex(vs);
        }

        int k = 0;
        for (int i = 0; i < triangles.size() / 3; i++) {
            /*FI(i, 0) = triangles[k++];
            FI(i, 1) = triangles[k++];
            FI(i, 2) = triangles[k++];*/
            int ind1 = triangles[k++];
            int ind2 = triangles[k++];
            int ind3 = triangles[k++];
            mesh_faces.create_triangle(ind1, ind2, ind3);
        }

        floatTetWild::Parameters arg;
        arg.ideal_edge_length_rel = params.ideal_edge_length;
        arg.max_its = params.max_passes;
        arg.is_quiet = true;
        //floatTetWild::tetrahedralization(sf_mesh, arg, VO, TO);
        //tetwild::tetrahedralization(VI, FI, VO, TO, AO, arg);
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

        constrained_face_count = 0;
        // Read constrained faces
        std::unordered_set<int> flags;
        in >> s >> t;
        for (int i = 0; i < tetra_count; i++) {
            for (int j = 0; j < 4; j++) {
                in >> s;
                int face_id = -1;
                if (s != "-") {
                    face_id = std::stoi(s) + 1;
                    flags.insert(face_id);
                    constrained_face_count++;
                }
                tets[i].face_idx[j] = face_id;
            }
        }
        std::cout << "Unique faces: " << flags.size() << std::endl;
        std::cout << "Constrained face count: " << constrained_face_count << std::endl;

        air_region_id = 0;

    }
};