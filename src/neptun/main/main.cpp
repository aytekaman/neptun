
#include <iomanip>
#include <iostream>

// #include <direct.h>

#include "bvh.h"
#include "filesystem.h"
#include "kd_tree.h"
#include "neptun/argparse/argparse.h"
#include "neptun/editor/editor.h"
#include "neptun/editor/graphics.h"
#include "procedural_mesh_generator.h"
#include "ray_tracer.h"
#include "scene.h"
#include "stats.h"
#include "tet_mesh.h"
#include "utils.h"

#include <glm/gtc/constants.hpp>

std::string colors[] = { "cyan", "orange", "red", "blue" };
std::string marks[] = { "square", "circle", "triangle", "plus" };

#ifdef _WIN32
    std::string builtin_scenes_folder_path = "../../scenes/";
#else
    std::string builtin_scenes_folder_path = "../sandbox/scenes/";
#endif


struct MethodInfo
{
    MethodInfo(Method method_, std::string name_)
    {
        method = method_;
        name = name_;
    }

    Method method;
    std::string name;
};

struct SceneInfo
{
    SceneInfo(std::string scene_name_, std::string short_name_, bool has_bounding_box_)
    {
        scene_name = scene_name_;
        short_name = short_name_;
        has_bounding_box = has_bounding_box_;
    }

    std::string scene_name;
    std::string short_name;
    bool has_bounding_box;
};

void create_and_render_test_scene()
{
    Scene scene;
    RayTracer ray_tracer;

    SceneObject* icosphere = new SceneObject("Icosphere");

    icosphere->mesh = ProceduralMeshGenerator::create_icosphere();
    scene.add_scene_object(icosphere);

    scene.build_tet_mesh(true, true);

    ray_tracer.Render(scene);

    std::cout << "simple test scene is rendered." << std::endl;
}

void build_and_render_times(const std::string folder_name, int N = 100)
{
    std::string test_folder_name = folder_name + "/build_and_render_times";
    fs::create_directory(test_folder_name);

    Scene scene;
    RayTracer ray_tracer;

    struct BenchmarkData
    {
        std::string scene_name;
        int triangle_count;
        std::vector<float> build_times;
        std::vector<float> render_times;
    };

    std::vector<SceneInfo> scene_infos =
    {
        SceneInfo("torus knots", "tk. 1", false),
        SceneInfo("torus knots in a box", "tk. 2", true),
        SceneInfo("armadillo", "armad. 1", false),
        SceneInfo("armadillo in a box", "armad. 2", true),
        SceneInfo("neptune", "neptune 1", false),
        SceneInfo("neptune in a box", "neptune 2", true),
        SceneInfo("mix", "mix 1", false),
        SceneInfo("mix in a box", "mix 2", true),
        SceneInfo("mix close", "mix 1 close", false),
        SceneInfo("mix in a box close", "mix 2 close", true),
    };

    //std::vector<SceneInfo> scene_infos =
    //{
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false),
    //	SceneInfo("Torus Knot", "TK. 1", false)
    //};

    std::string test_dir = "./Assets/Test Scenes/";

    std::vector<std::string> accelerators =
    {
        "kd-tree",
        "BVH",
        "tet-mesh",
    };

    std::vector<MethodInfo> methodInfos =
    {
        MethodInfo(Method::ScTP, "ScTP"),
        //MethodInfo(Method::Default, "Our method"),
        MethodInfo(Method::Fast_basis, "Fast basis"),
        MethodInfo(Method::Kd_tree, "kd-tree"),
        MethodInfo(Method::BVH_pbrt, "BVH")
    };

    std::cout << "Method count: " << methodInfos.size() << std::endl;

    std::vector<BenchmarkData> benchmark_datas;

    for (int i = 0; i < scene_infos.size(); ++i)
    {
        BenchmarkData benchmark_data;
        benchmark_data.scene_name = scene_infos[i].scene_name;

        std::string path = "./Assets/";
        path.append(scene_infos[i].scene_name);
        path.append(".scene");

        benchmark_data.render_times.resize(methodInfos.size(), 0);

        scene.load_from_file(path);

        for (int j = 0; j < accelerators.size(); ++j)
        {
            if (accelerators[j] == "kd-tree")
                scene.build_kd_tree();
            else if (accelerators[j] == "BVH")
                scene.build_bvh();
            else if (accelerators[j] == "tet-mesh")
                scene.build_tet_mesh(true, !scene_infos[i].has_bounding_box);//  tet_mesh = new TetMesh(scene, true, !scene_infos[i].has_bounding_box, 5.0);  // build_tet_mesh(true, false, 5.0f);

            benchmark_data.build_times.push_back(Stats::last_build_time);
        }

        if (scene.tet_mesh)
            scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, true);

        benchmark_data.triangle_count = scene.get_triangle_count();

        for (int k = 0; k < N; ++k)
        {
            for (int j = 0; j < methodInfos.size(); ++j)
            {
                ray_tracer.method = methodInfos[j].method;
                ray_tracer.Render(scene);

                benchmark_data.render_times[j] += ray_tracer.last_render_time;
            }
        }

        std::string file_name = test_folder_name + "/" + scene_infos[i].scene_name + ".png";

        ray_tracer.save_to_disk(file_name.c_str());

        benchmark_datas.push_back(benchmark_data);
    }

    std::ofstream file(test_folder_name + "/table.tex");

    // ------------------------------------------------------------------------
    // Latex Figures
    // ------------------------------------------------------------------------

    file << "\\begin{table*}[t]" << std::endl;
    file << "\\centering" << std::endl;
    file << "\\begin{tabular}{|lrrrrr|}" << std::endl;

    int row_count = 2;

    for (int row_index = 0; row_index < row_count; row_index++)
    {
        file << "\\hline" << std::endl;
        file << "\\multicolumn{6}{|c|}{Scenes} \\\\" << std::endl;
        file << "\\hline" << std::endl;

        for (int i = row_index * 5; i < row_index * 5 + 5; i++)
            file << "& " << benchmark_datas[i].scene_name << " ";
        file << "\\\\" << std::endl;

        for (int i = row_index * 5; i < row_index * 5 + 5; i++)
            file << "& " << "\\includegraphics[width = 0.14\\textwidth]{{\"build_and_render_times/" << benchmark_datas[i].scene_name << "\"}.png} ";
        file << "\\\\" << std::endl;

        file << "\\hline" << std::endl;
        file << "\\multicolumn{6}{}{}\\\\";
        file << "\\hline" << std::endl;

        file << "\\multicolumn{6}{|c|}{Scene stats} \\\\" << std::endl;
        file << "\\hline" << std::endl;

        file << "\\# of triangles ";
        for (int i = row_index * 5; i < row_index * 5 + 5; i++)
            file << "& " << benchmark_datas[i].triangle_count << " ";
        file << "\\\\" << std::endl;

        file << "\\hline" << std::endl;
        file << "\\multicolumn{6}{}{}\\\\";
        file << "\\hline" << std::endl;

        file << std::fixed;
        file.precision(2);

        file << "\\multicolumn{6}{|c|}{Construction times} \\\\" << std::endl;
        file << "\\hline" << std::endl;

        for (int j = 0; j < accelerators.size(); j++)
        {
            file << accelerators[j];
            for (int i = row_index * 5; i < row_index * 5 + 5; i++)
            {
                bool best = true;

                for (int k = 0; k < accelerators.size(); k++)
                {
                    if (benchmark_datas[i].build_times[j] > benchmark_datas[i].build_times[k])
                        best = false;
                }

                if (best)
                    file << "& \\textbf{" << benchmark_datas[i].build_times[j] << "} ";
                else
                    file << "& " << benchmark_datas[i].build_times[j] << " ";
            }

            file << "\\\\" << std::endl;
            //file << "\\hline" << std::endl;
        }

        file << "\\hline" << std::endl;
        file << "\\multicolumn{6}{}{}\\\\";
        file << "\\hline" << std::endl;

        file.precision(4);

        file << "\\multicolumn{6}{|c|}{Render times} \\\\" << std::endl;
        file << "\\hline" << std::endl;

        for (int j = 0; j < methodInfos.size(); j++)
        {
            file << methodInfos[j].name << " ";
            for (int i = row_index * 5; i < row_index * 5 + 5; i++)
            {
                bool best = true;

                for (int k = 0; k < methodInfos.size(); k++)
                {
                    if (benchmark_datas[i].render_times[j] > benchmark_datas[i].render_times[k])
                        best = false;
                }

                if (best)
                    file << "& \\textbf{" << benchmark_datas[i].render_times[j] / N << "} ";
                else
                    file << "& " << benchmark_datas[i].render_times[j] / N << " ";
            }

            file << "\\\\" << std::endl;
            //file << "\\hline" << std::endl;
        }

        file << "\\hline" << std::endl;
        file << "\\multicolumn{6}{}{}\\\\";
    }


    //file << "\\hline" << std::endl;
    //file << "\\multicolumn{6}{}{}\\\\";
    //file << "\\hline" << std::endl;

    file << "\\end{tabular}" << std::endl;
    file << "\\caption{Construction times for different acceleration structures and render times for different traversal methods.}" << std::endl;
    file << "\\label{tab:build_and_render_times}" << std::endl;
    file << "\\end{table*}" << std::endl;
}

void close_surface(const std::string folder_name, int N = 100)
{
    std::string test_folder_name = folder_name + "/close_surface";
    fs::create_directory(test_folder_name);

    std::vector<MethodInfo> methodInfos =
    {
        MethodInfo(Method::Fast_basis, "tet-mesh"),
        MethodInfo(Method::Kd_tree, "kd-tree"),
        MethodInfo(Method::BVH_pbrt, "BVH")
    };

    Scene scene;
    scene.camOrbitX = 0;
    scene.camOrbitY = glm::pi<float>() / 2.0f;

    RayTracer ray_tracer;

    scene.load_from_file("./Assets/Armadillo Zoom.scene");

    scene.build_bvh();
    scene.build_kd_tree();
    scene.build_tet_mesh(true, true);

    scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, true);

    struct BenchmarkData
    {
        std::vector<float> costs;
        std::vector<float> times;
    };

    std::vector<BenchmarkData> benchmark_datas;

    for (int i = 0; i < 6; i++)
    {
        BenchmarkData benchmark_data;

        for (int j = 0; j < methodInfos.size(); j++)
        {
            ray_tracer.method = methodInfos[j].method;
            ray_tracer.Render(scene, true);

            std::string file_name = test_folder_name + "/zoom_" + methodInfos[j].name + "_" + std::to_string(i) + ".png";

            ray_tracer.save_to_disk(file_name.c_str(), ImageType::Tet_cost);
            benchmark_data.costs.push_back(ray_tracer.avg_test_count);

            benchmark_data.times.resize(methodInfos.size());

            for (int k = 0; k < N; ++k)
            {
                ray_tracer.Render(scene);
                benchmark_data.times[j] += ray_tracer.last_render_time;;
            }
        }

        benchmark_datas.push_back(benchmark_data);
        scene.camDist -= 2.6f;
    }

    // ------------------------------------------------------------------------
    // Latex Figures
    // ------------------------------------------------------------------------

    std::ofstream file(test_folder_name + "/table.tex");

    file << "\\begin{table*}[t]" << std::endl;
    file << "\\centering" << std::endl;
    file << "\\begin{tabular}{|lrrrrrr|}" << std::endl;
    file << "\\hline" << std::endl;
    file << "\\multicolumn{7}{|c|} {Scenes} \\\\" << std::endl;
    file << "\\hline" << std::endl;

    for (int j = 0; j < methodInfos.size(); j++)
    {
        file << methodInfos[j].name;
        for (int i = 0; i < benchmark_datas.size(); ++i)
            file << " & " << "\\includegraphics[width = 0.12\\textwidth]{close_surface/zoom_" << methodInfos[j].name << "_" << i << ".png}";
        file << "\\\\" << std::endl;
        //file << "\\hline" << std::endl;
    }

    file << "\\hline" << std::endl;
    // !!! 7 is hardcoded
    file << "\\multicolumn{7}{}{} \\\\" << std::endl;


    file << "\\hline" << std::endl;
    file << "\\multicolumn{7}{|c|} {Visited node count per pixel} \\\\" << std::endl;
    file << "\\hline" << std::endl;

    file << std::fixed;
    file.precision(2);

    for (int j = 0; j < methodInfos.size(); j++)
    {
        file << methodInfos[j].name;
        for (int i = 0; i < benchmark_datas.size(); ++i)
        {
            bool best = true;

            for (int k = 0; k < methodInfos.size(); k++)
            {
                if (benchmark_datas[i].costs[j] > benchmark_datas[i].costs[k])
                    best = false;
            }

            if (best)
                file << " & \\textbf{" << benchmark_datas[i].costs[j] << "}";
            else
                file << " & " << benchmark_datas[i].costs[j];
        }

        file << "\\\\" << std::endl;
        //file << "\\hline" << std::endl;
    }

    file << "\\hline" << std::endl;

    // !!! 7 is hardcoded
    file << "\\multicolumn{7}{}{} \\\\" << std::endl;
    file << "\\hline" << std::endl;
    file << "\\multicolumn{7}{|c|} {Render times (in seconds)} \\\\" << std::endl;
    file << "\\hline" << std::endl;

    file.precision(4);

    for (int j = 0; j < methodInfos.size(); j++)
    {
        file << methodInfos[j].name;

        for (int i = 0; i < benchmark_datas.size(); ++i)
        {
            bool best = true;

            for (int k = 0; k < methodInfos.size(); k++)
            {
                if (benchmark_datas[i].times[j] > benchmark_datas[i].times[k])
                    best = false;
            }

            if (best)
                file << " & \\textbf{" << benchmark_datas[i].times[j] / N << "}";
            else
                file << " & " << benchmark_datas[i].times[j] / N;
        }

        file << "\\\\" << std::endl;
        //file << "\\hline" << std::endl;
    }

    file << "\\hline" << std::endl;

    file << "\\end{tabular}" << std::endl;
    file << "\\caption{Rendering times and visited node counts as camera gets closer to the mesh surface for different type of accelerators.}" << std::endl;
    file << "\\label{ tab:close_surface }" << std::endl;
    file << "\\end{table*}" << std::endl;
}

void tet_mesh_weight(const std::string folder_name, int N = 100)
{
    std::string test_folder_name = folder_name + "/tet_mesh_weight";
    fs::create_directory(test_folder_name);

    std::vector<SceneInfo> scene_infos =
    {
        //SceneInfo("Torus Knots", "TK. 1", false),
        //SceneInfo("Armadillo", "Armad. 1", false),
        //SceneInfo("Armadillo in a Box", "Armad. 2", true),
        //SceneInfo("Torus Knot", "TK. 1", false),

        //SceneInfo("Neptune", "Neptune 1", false),
        SceneInfo("Mix", "Mix", false),
        //SceneInfo("Torus Knots in a Box", "TK. 2", true)
    };

    float q_min = 2.0f;
    float q_max = 10.f;
    int step_count = 15;

    float q = q_min;


    Scene scene;
    //scene.load_from_file("./Assets/Torus Knots.scene");

    RayTracer ray_tracer;

    struct BenchmarkData
    {
        std::string scene_name;

        std::vector<float> qs;
        std::vector<float> weights;
        std::vector<float> costs;
        std::vector<float> times;
    };

    std::vector<BenchmarkData> benchmark_datas;

    char file_name[128];

    for (int i = 0; i < scene_infos.size(); i++)
    {
        BenchmarkData benchmark_data;
        benchmark_data.scene_name = scene_infos[i].scene_name;

        std::string path = "./Assets/";
        path.append(scene_infos[i].scene_name);
        path.append(".scene");

        scene.load_from_file(path);

        float q = q_min;

        for (int j = 0; j < step_count; j++)
        {
            scene.build_tet_mesh(true, !scene_infos[i].has_bounding_box, q);

            std::cout << j << std::endl;

            scene.tet_mesh->compute_weight();

            benchmark_data.qs.push_back(q);
            benchmark_data.weights.push_back(scene.tet_mesh->m_weight);
            ray_tracer.Render(scene, true);


            //sprintf_s(file_name, 128, "tet_cost_q_%.1f.png", q);

            // also print tet count

            ray_tracer.save_to_disk(file_name, ImageType::Tet_cost);
            benchmark_data.costs.push_back(ray_tracer.avg_test_count);

            //for (int j = 0; j < N; ++j)
            //{
            //    ray_tracer.Render(scene, false);
            //    benchmark_data.time += ray_tracer.last_render_time;
            //}

            q += 0.5f;
        }

        benchmark_datas.push_back(benchmark_data);
    }

    // ------------------------------------------------------------------------
    // Latex Figures
    // ------------------------------------------------------------------------

    std::ofstream file(test_folder_name + "/figure.tex");

    file << "\\begin{tikzpicture}" << std::endl;
    file << "\\begin{axis}[" << std::endl;
    file << "    title = { Weight of a tetrahedral mesh }," << std::endl;
    file << "        xlabel = { Maximum radius - edge ratio }," << std::endl;
    file << "        ylabel = { Weight }," << std::endl;
    file << "        xmin = 2.0, xmax = 9.0," << std::endl;
    file << "        ymin = 0, ymax = 1800," << std::endl;
    file << "        xtick = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 }," << std::endl;
    file << "        ytick = { 0, 450, 900, 1350, 1800 }," << std::endl;
    file << "        legend pos = south west," << std::endl;
    file << "        ymajorgrids = true," << std::endl;
    file << "        grid style = dashed," << std::endl;
    file << "        width = \\columnwidth," << std::endl;
    file << "]" << std::endl;

    for (int i = 0; i < benchmark_datas.size(); ++i)
    {
        file << "\\addplot[color = " << colors[i] << ", mark = triangle,] coordinates{" << std::endl;

        for (int j = 0; j < benchmark_datas[i].qs.size(); j++)
        {
            file << "(" << benchmark_datas[i].qs[j] << ", " << benchmark_datas[i].weights[j] / 1000 << ")" << std::endl;
        }

        file << "};" << std::endl;
    }

    file << "\\legend{ ";

    for (int i = 0; i < benchmark_datas.size(); ++i)
    {
        file << benchmark_datas[i].scene_name;
        if (i == benchmark_datas.size() - 1)
            file << "}" << std::endl;
        else
            file << ", ";
    }

    file << "\\end{axis}" << std::endl;
    file << "\\end{tikzpicture}" << std::endl;
}

void tet_mesh_sorting(const std::string folder_name, int N = 1000)
{
    std::string test_folder_name = folder_name + "/tet_mesh_sorting";

    fs::create_directory(test_folder_name);

    std::vector<SceneInfo> scene_infos =
    {
        SceneInfo("Torus Knots", "TKs. 1", false),
        SceneInfo("Armadillo", "Armad. 1", false),
        //SceneInfo("Torus Knot", "TK. 1", false),

        //SceneInfo("Torus Knots", "TK. 1", false),
        //SceneInfo("Torus Knots", "TK. 1", false),
        //SceneInfo("Torus Knots", "TK. 1", false),
        SceneInfo("Neptune", "Neptune 1", false),
        SceneInfo("Mix", "Mix 1", false),
        SceneInfo("Mix close", "Mix 1 close", false),
    };

    struct SortingMethodInfo
    {
        SortingMethodInfo(SortingMethod sorting_method_, bool use_regions_, std::string name_)
        {
            sorting_method = sorting_method_;
            use_regions = use_regions_;
            name = name_;
        }

        SortingMethod sorting_method;
        bool use_regions;
        std::string name;
    };

    std::vector<SortingMethodInfo> sorting_method_infos =
    {
        SortingMethodInfo(SortingMethod::Default, false, "Unsorted"),
        SortingMethodInfo(SortingMethod::Hilbert, false, "Hilbert"),
        SortingMethodInfo(SortingMethod::Hilbert, true, "Hilbert (regions)"),
    };

    struct BenchmarkData
    {
        std::vector<float> render_times;
    };

    std::vector<BenchmarkData> benchmark_datas;

    RayTracer ray_tracer;
    ray_tracer.method = Method::Fast_basis;

    for (int i = 0; i < scene_infos.size(); ++i)
    {
        Scene scene;

        std::string path = "./Assets/";
        path.append(scene_infos[i].scene_name);
        path.append(".scene");

        scene.load_from_file(path);

        BenchmarkData benchmark_data;

        for (int j = 0; j < sorting_method_infos.size(); ++j)
        {
            scene.build_tet_mesh(true, true);
            scene.tet_mesh->sort(sorting_method_infos[j].sorting_method, 16U, sorting_method_infos[j].use_regions);

            for (int k = 0; k < N; ++k)
            {
                ray_tracer.Render(scene);
            }

            benchmark_data.render_times.push_back(Stats::get_avg_render_time(N));
        }

        benchmark_datas.push_back(benchmark_data);
    }

    std::ofstream file(test_folder_name + "/figure.tex");

    file << "\\begin{figure}" << std::endl;
    file << "\\begin{tikzpicture}" << std::endl;
    file << "\\begin{axis}[" << std::endl;
    file << "title = {Performance comparison of different tet-mesh sorting methods}," << std::endl;
    file << "ybar = 0.6," << std::endl;
    file << "ymin = 0," << std::endl;
    file << "enlarge x limits=0.2," << std::endl;
    file << "enlarge y limits={upper, value=0.2}," << std::endl;
    file << "ylabel = Render time (sec.)," << std::endl;
    file << "legend style = { at = { (0.5,0.95) }," << std::endl;
    file << "anchor = north,legend columns = -1 }," << std::endl;
    file << "width = \\columnwidth," << std::endl;
    file << "xticklabel style={rotate=45}," << std::endl;
    file << "symbolic x coords={";
    for (int i = 0; i < scene_infos.size(); ++i)
        file << scene_infos[i].scene_name << ", ";
    file << "}," << std::endl;

    file << "xtick = data," << std::endl;

    file << "]" << std::endl;

    for (int i = 0; i < sorting_method_infos.size(); i++)
    {
        file << "\\addplot" << std::endl;
        file << "coordinates{" << std::endl;

        for (int j = 0; j < scene_infos.size(); j++)
        {
            file << "(" << scene_infos[j].scene_name << "," << benchmark_datas[j].render_times[i] << ")" << std::endl;
        }

        file << "};" << std::endl;
    }

    file << "\\legend{" << std::endl;

    for (int i = 0; i < sorting_method_infos.size(); i++)
        file << sorting_method_infos[i].name << ", ";

    file << "}" << std::endl;

    file << "\\end{axis}" << std::endl;
    file << "\\end{tikzpicture}" << std::endl;
    file << "\\caption{Render time comparison for unsorted vs. sorted tet-mesh data.}" << std::endl;
    file << "\\end{figure}" << std::endl;
}

void region_sort(int N = 5000)
{
    RayTracer ray_tracer;

    Scene scene;
    scene.load_from_file("Assets/Mix.scene");
    scene.build_tet_mesh(true, true);
    scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, false);

    Scene scene2;
    scene2.load_from_file("Assets/Mix.scene");
    scene2.build_tet_mesh(true, true);
    scene2.tet_mesh->sort(SortingMethod::Hilbert, 16U, true);

    float render_time_with_regions = 0.0f;
    float render_time_without_regions = 0.0f;

    for (int i = 0; i < N; i++)
    {
        ray_tracer.Render(scene);
        render_time_without_regions += ray_tracer.last_render_time;

        ray_tracer.Render(scene2);
        render_time_with_regions += ray_tracer.last_render_time;
    }

    //render_time_without_regions = Stats::get_avg_render_time(N);

    //for (int i = 0; i < N; i++)
    //{
    //    ray_tracer.Render(scene);
    //}

    //render_time_with_regions = Stats::get_avg_render_time(N);

    std::cout << render_time_without_regions / N << std::endl;
    std::cout << render_time_with_regions / N << std::endl;
}

void simd_comparison()
{
    RayTracer ray_tracer;
    Scene scene;
    scene.load_from_file(builtin_scenes_folder_path + "Armadillo.scene");
    scene.build_tet_mesh(true, true);
    scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, false);

    int N = 1000;

    float non_simd_fps = 0.0f;
    float simd_fps = 0.0f;

    for (int k = 0; k < N; ++k)
    {
        ray_tracer.method = Method::Default;
        ray_tracer.Render(scene);
        non_simd_fps = glm::max(1.0f / ray_tracer.last_render_time, non_simd_fps);

        ray_tracer.method = Method::DefaultSimd;
        ray_tracer.Render(scene);
        simd_fps = glm::max(1.0f / ray_tracer.last_render_time, simd_fps);
    }

    //non_simd_fps /= N;
    //simd_fps /= N;

    std::cout << "non-simd: " << non_simd_fps << std::endl;
    std::cout << "    simd: " << simd_fps << std::endl;

    std::cout << "difference: " << glm::abs(non_simd_fps - simd_fps) / non_simd_fps;
}

#ifdef _WIN32
extern "C"
{
	_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}
#endif

int run_command_line(const argparse::ArgumentData& args)
{
    if (args["help"]->cast<bool>())
    {
        args.print_usage();
        return EXIT_SUCCESS;
    }

    if (args.has_errors())
    {
        args.print_errors();
    }
    args.print_warnings();

    return EXIT_SUCCESS;
}

int main(int argc, char const *argv[])
{
    if (argc > 1)
    {
    /*
        std::cout << "Starting in benchmark mode ... " << std::endl;
        create_and_render_test_scene();
        simd_comparison();
    */
        const std::string program_name(argv[0]);
        argparse::ArgumentParser parser(program_name, "Neptun command line interface", run_command_line);

        parser.add_keyword_argument("help", "Prints help", argparse::ArgumentType::BOOL, "h", "false");

        if (parser.has_errors())
        {
            std::cerr << "Parser error" << std::endl;
            return EXIT_FAILURE;
        } else
        {
            return parser.parse(argc - 1, argv + 1);
        }
    }
    else
    {
        Scene scene;
        Graphics graphics;
        RayTracer ray_tracer;

        Editor editor(&scene, &graphics, &ray_tracer);

        editor.Run();
        return EXIT_SUCCESS;
    }
}
