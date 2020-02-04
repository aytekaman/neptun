
#include <iomanip>
#include <iostream>

// #include <direct.h>

#include "bvh.h"
#include "filesystem.h"
#include "image.h"
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

// Util functions
bool file_exists(const std::string& file_path)
{
    std::ifstream infile(file_path);
    return infile.good();
}

bool parse_resolution(const std::string& res, std::size_t& width, std::size_t& height)
{
    std::stringstream ss(res);
    char c;
    ss >> width >> c >> height;

    if (ss.fail() || width == 0 || height == 0 || (c != 'x' && c != ','))
        return false;

    if (ss.peek() != EOF)
    {
        return ((ss >> std::ws).fail() == false) && (ss.peek() == EOF);
    }

    return true;
} 

// Commands
int command_render_scene(const argparse::ArgumentData& args)
{
    args.print_warnings();
    if (args.has_errors())
    {
        args.print_errors();
        args.print_usage();
        return EXIT_FAILURE;
    }

    if (args["help"]->cast<bool>())
    {
        args.print_usage();
        return EXIT_SUCCESS;
    }

    const std::string scene_file = args["scene"]->value();
    const std::string output_file = args["output"]->value();
    const std::string rendering_method = args["accelerator"]->value();
    const bool diagnostic = args["diagnostic"]->cast<bool>();
    const std::string output_base = output_file.substr(0, output_file.find_last_of('.'));

    // Parse resolution pair
    const std::string output_resolution = args["resolution"]->value();

    // std::cout << output_resolution << std::endl;

    size_t image_width, image_height;

    if (parse_resolution(output_resolution, image_width, image_height) == false)
    {
        // Parse error
        std::cerr << "Incorrect resolution format" << std::endl;
        args.print_usage();
        return EXIT_FAILURE;
    }

    if (output_file.substr(output_file.size() - 4) != ".png")
    {
        std::cerr << "Output file must end with \".png\"\n";
        return EXIT_FAILURE;
    }

    if (file_exists(scene_file) == false)
    {
        std::cerr << "Cannot find scene file" << std::endl;
        return EXIT_FAILURE;
    }

    Scene scene;
    RayTracer ray_tracer;
    scene.load_from_file(scene_file);

    size_t accelerator_size_in_bytes = 0;

    if (rendering_method.find("tet-mesh") != std::string::npos)
    {
        const bool use_cache = args["usecache"]->cast<bool>();
        const float quality = args["quality"]->cast<float>();
        const bool preserve_triangles = !args["split-triangles"]->cast<bool>();

        // Limit quality
        if (quality < 1 || quality > 25)
        {
            std::cerr << "Quality must be in range [1, 25]\n";
            return EXIT_FAILURE;
        }

        if(rendering_method == "tet-mesh-16")
            scene.tet_mesh = use_cache ? new TetMesh16(scene) : new TetMesh16(scene, preserve_triangles, true, quality);
        else if (rendering_method == "tet-mesh-20")
            scene.tet_mesh = use_cache ? new TetMesh20(scene) : new TetMesh20(scene, preserve_triangles, true, quality);
        else if (rendering_method == "tet-mesh-32")
            scene.tet_mesh = use_cache ? new TetMesh32(scene) : new TetMesh32(scene, preserve_triangles, true, quality);
        else if (rendering_method == "tet-mesh-80")
            scene.tet_mesh = use_cache ? new TetMesh80(scene) : new TetMesh80(scene, preserve_triangles, true, quality);
        else if (rendering_method == "tet-mesh-sctp")
            scene.tet_mesh = use_cache ? new TetMeshSctp(scene) : new TetMeshSctp(scene, preserve_triangles, true, quality);
        else
        {
            std::cerr << "Unrecognized rendering method " << rendering_method << std::endl;
            return EXIT_FAILURE;
        }

        ray_tracer.method = Method::Default;

        accelerator_size_in_bytes = scene.tet_mesh->get_size_in_bytes();

        const std::string sorting_method = args["sorting"]->value();

        if(sorting_method == "hilbert")
            scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, false);
        else if (sorting_method == "hilbert-regions")
            scene.tet_mesh->sort(SortingMethod::Hilbert, 16U, true);
        else if (sorting_method == "none");
        else
        {
            std::cerr << "Unrecognized sorting method " << sorting_method << std::endl;
            return EXIT_FAILURE;
        }
    } 
    else if (rendering_method == "bvh")
    {
        scene.build_bvh();
        accelerator_size_in_bytes = scene.bvh->get_size_in_bytes();
        ray_tracer.method = Method::BVH_pbrt;
    }
    else if (rendering_method == "kd")
    {
        scene.build_kd_tree();
        accelerator_size_in_bytes = scene.kd_tree->get_size_in_bytes();
        ray_tracer.method = Method::Kd_tree;
    }
    else if (rendering_method == "embree") 
    {
        scene.build_bvh_embree();
        accelerator_size_in_bytes = 0; // Not supported yet.
        ray_tracer.method = Method::BVH_embree;
    }
    else 
    {
        std::cerr << "Unrecognized rendering method " << rendering_method << std::endl;
        return EXIT_FAILURE;
    }

    ray_tracer.set_resolution(glm::ivec2(image_width, image_height));

    if (args["path-tracing"]->cast<bool>())
    {
        ray_tracer.integrator = Integrator::PathTracing;
    }

    const int N = args["repetition"]->cast<int>();

    for(int i = 0; i < N; ++i)
        ray_tracer.Render(scene, diagnostic);

    std::cout << std::fixed << std::setprecision(5); // Set precision and print format for floating point numbers
    std::cout << "triangle_count=" << scene.get_triangle_count() << std::endl;
    std::cout << "build_time=" << Stats::last_build_time << std::endl;
    std::cout << "render_time=" << 1000. * Stats::get_best_render_time(N) << std::endl;
    std::cout << "visited_node_count=" << ray_tracer.avg_test_count << std::endl; // avg
    std::cout << "size_in_bytes=" << accelerator_size_in_bytes << std::endl;

    ray_tracer.m_rendered_image->save_to_disk(output_file.c_str());
    std::cout << "Rendered image saved at : " << output_file << std::endl;

    if (diagnostic)
    {
        const std::string diag_filename = output_base + "_diag.png";
        ray_tracer.m_visited_tets_image->save_to_disk(diag_filename.c_str());
        
        std::cout << "Diagnostic image saved at : " << output_file << std::endl;
    }

    return EXIT_SUCCESS;
}

int run_editor()
{
    Scene scene;
    Graphics graphics;
    RayTracer ray_tracer;

    Editor editor(&scene, &graphics, &ray_tracer);

    editor.Run();
    return EXIT_SUCCESS;
}

int run_command_line(int argc, char const* argv[])
{
    struct Command
    {
        std::string name;
        std::string desc;
    };

    if (argc < 2)
    {
        return EXIT_FAILURE;
    }

    const std::string program_name(argv[0]);
    const std::string command_name(argv[1]);

    const std::vector<Command> commands = {
        {"list", "Lists all available commands"},
        {"editor", "Runs editor"},
        {"simd_benchmark", "Benchmark simd intersection"},
        {"render", "Renders a scene"}
    };

    auto command_it = std::find_if(commands.begin(), commands.end(), [command_name](const Command& c){ return c.name == command_name; });

    // Command not found
    if (command_it == commands.end() || command_it->name == "list")
    {
        // Print available commands
        std::ostringstream help_text;

        if (command_it == commands.end())
            help_text << "Unrecognized command " << command_name << "\n";

        help_text << "Usage: " << program_name << " <command> \n"
                  << "Commands:\n"
                  << std::left; 
        
        for (Command c : commands)
        {
            help_text << "  " << std::setw(20) << c.name << std::setw(0) << c.desc << "\n";
        }

        std::cout << help_text.str();
        return EXIT_FAILURE;
    }

    Command command = *command_it;
    if (command.name == "editor")
    {
        return run_editor();
    } 
    else if (command.name == "simd_benchmark")
    {
        std::cout << "Starting in benchmark mode ... " << std::endl;
        create_and_render_test_scene();
        simd_comparison();

        return EXIT_SUCCESS;
    }
    else if (command.name == "render")
    {
        using argparse::ArgumentType;
        argparse::ArgumentParser parser(program_name + " render",
                                        "render given scene",
                                        command_render_scene);
        
        parser.add_positional_argument("scene", "Scene file to be rendered")
            .add_keyword_argument("accelerator", "Accelerator type used for intersections. (tet-mesh-32, bvh, kd, embree)", ArgumentType::STRING, "m", "tet-mesh-32")
            .add_keyword_argument("output", "Output file", ArgumentType::STRING, "o", "a.png")
            .add_keyword_argument("resolution", "Resolution of the output file", ArgumentType::STRING, "r", "1920x1440")
            .add_keyword_argument("help", "Prints help", ArgumentType::BOOL, "h")
            .add_keyword_argument("diagnostic", "Output diagnostic image", ArgumentType::BOOL, "d")
            .add_keyword_argument("repetition", "Number of repetitions", ArgumentType::INTEGER, "n", "100")
            .add_keyword_argument("sorting", "Sorting method for tet-mesh-32. (hilbert-regions, hilbert, none)", ArgumentType::STRING, "s", "hilbert")
            .add_keyword_argument("usecache", "Use .tetmesh cache file", ArgumentType::BOOL, "c", "false")
            .add_keyword_argument("quality", "Tetmesh quality (Minimum radius-edge ratio)", ArgumentType::FLOAT, "q", "5.0")
            .add_keyword_argument("split-triangles", "Allow triangles to be splitted in tetmesh. (!preserve triangles)", ArgumentType::BOOL, "y")
            .add_keyword_argument("path-tracing", "Use path tracing", ArgumentType::BOOL, "p");
              

        return parser.parse(argc - 2, argv + 2);
    }

    return EXIT_SUCCESS;
}

int main(int argc, char const* argv[])
{
    if (argc > 1)
    {
        run_command_line(argc, argv);
    }
    else
    {
        run_editor();
    }
}
