import locale
import os
import re
import subprocess
import time

neptun_exe_dir_path = "C:/dev/neptun/sandbox/bin/Release/"
neptun_exe_path = "C:/dev/neptun/sandbox/bin/Release/neptun.exe"

scene_dir_path = "C:/dev/neptun/sandbox/scenes/"

original_scene_names = [
    "Mix",]

original_scene_names_without_box = [
    "Torus Knots",
    "Armadillo",
    "Neptune",
    "Mix",
    "Mix close", ]

def benchmar_tetmesh_traversal():

    scene_names = original_scene_names

    methods = [
        "tet-mesh-20",
        "tet-mesh-16",
        ]

    N = 128

    print("Tetmesh traversal benchmark is started.")

    benchmark_data = {}

    experiment_count = len(scene_names) * len(methods)
    experiment_index = 0

    for scene_name in scene_names:

        benchmark_data[scene_name] = {}
        benchmark_data[scene_name]["build_times"] = {}
        benchmark_data[scene_name]["render_times"] = {}

        for method in methods:
            scene_path = '"' + scene_dir_path + scene_name + ".scene" + '"'
            result = subprocess.run(
                neptun_exe_path + " render " + scene_path + " -m " + method + " -n " +
                str(N) + 
                " -c",
                cwd=neptun_exe_dir_path,
                stdout=subprocess.PIPE)
            output = result.stdout.decode('utf-8')

            experiment_index += 1

            # Progress bar
            print(str(experiment_index) + "/" + str(experiment_count),  end = '\r', flush = True)

            triangle_count = int(get_value(output, "triangle_count"))
            build_time = float(get_value(output, "build_time"))
            render_time = float(get_value(output, "render_time"))

            benchmark_data[scene_name]["triangle_count"] = triangle_count
            benchmark_data[scene_name]["build_times"][method] = build_time
            benchmark_data[scene_name]["render_times"][method] = render_time

            time.sleep(1.0)

    print(benchmark_data)
    #print(os.getcwd())

    print("Memory usage experiments started.")

    #scene_names = original_scene_names_without_box
    #scene_names.pop()

    scene_names = [
        "Torus Knot",
        "Torus Knots",
        "Armadillo",
        "Neptune",
        "Mix",]

    methods = [
        "tet-mesh-32",
        "tet-mesh-20",
        "tet-mesh-16",
        "tet-mesh-80",]

    N = 1

    experiment_dir = "memory_usage"
    output_dir_path = os.path.dirname(
        os.path.realpath(__file__)) + "/" + experiment_dir

    benchmark_data = {}

    experiment_count = len(scene_names) * len(methods)
    experiment_index = 0

    for i in range(0, len(scene_names)):
        scene_name = scene_names[i]

        benchmark_data[scene_name] = {}
        benchmark_data[scene_name]["triangle_count"] = {}
        benchmark_data[scene_name]["size_in_bytes"] = {}
        benchmark_data[scene_name]["render_times"] = {}

        for method in methods:
            scene_path = '"' + scene_dir_path + scene_name + ".scene" + '"'
            result = subprocess.run(
                neptun_exe_path +
                " render " + scene_path +
                " -o " + '"' + output_dir_path + "/" + scene_name + ".png" + '"' +
                " -m " + method +
                " -n " + str(N) +
                " -c",
                cwd=neptun_exe_dir_path,
                stdout=subprocess.PIPE)
            output = result.stdout.decode('utf-8')

            experiment_index += 1
            # Progress bar
            print(str(experiment_index) + "/" + str(experiment_count),  end = '\r', flush = True)

            triangle_count = int(get_value(output, "triangle_count"))
            size_in_bytes = int(get_value(output, "size_in_bytes"))  / (1024.0 * 1024.0)
            render_time = float(get_value(output, "render_time"))

            benchmark_data[scene_name]["triangle_count"] = triangle_count
            benchmark_data[scene_name]["size_in_bytes"][method] = size_in_bytes
            benchmark_data[scene_name]["render_times"][method] = render_time

    print(benchmark_data)
    
def main():
    benchmar_tetmesh_traversal()
    #experiment_tetmesh_sorting()
    #experiment_tetmesh_weight()
    #experiment_camera_distance()
    #experiment_memory_usage()

# Taken from appleseed
def get_value(output, key):
    pattern = r"^{0}=(.*)[\r\n]+$".format(key)
    match = re.search(pattern, output, re.MULTILINE)
    return match.group(1) if match else None

main()
