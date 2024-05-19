# Neptun Renderer

Experimental renderer focused on tetrahedral mesh surface acceleration structure research.

<table>
    <tr>
        <td><img src="https://github.com/aytekaman/neptun/raw/master/resources/neptun-editor-00.png" height="%50" /></td> 
        <td><img src="https://github.com/aytekaman/neptun/raw/master/resources/neptun-editor-01.png" height="%50" /></td>
    </tr>
</table>

Neptun supports the following acceleration structures,

- Tetmesh32, Tetmesh20, Tetmesh16 [[A]](#w-a)
- TetmeshSctp [[1]](#r-1)
- Tetmesh80 [[2]](#r-2)
- Pbrt BVH, Kd-tree [[3]](#r-3)
- Embree BVH [[4]](#r-4)

# Setup

```shell
mkdir build
cd build
cmake .. -Dembree_DIR=/dir/to/embree
make all
```

# Third-Party Libraries

Repository contains various third-party libraries,

- [gl3w](https://github.com/skaslev/gl3w)
- [glfw](https://github.com/glfw/glfw)
- [glm](https://github.com/g-truc/glm)
- [Dear ImGui](https://github.com/ocornut/imgui)
- [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo)
- [noc](https://github.com/guillaumechereau/noc)
- [stb](https://github.com/nothings/stb)
- [tinyobjloader](https://github.com/tinyobjloader/tinyobjloader)
- [tetgen](https://wias-berlin.de/software/index.jsp?id=TetGen&lang=1)
- [hilbert](https://web.archive.org/web/20041028171141/http://www.caam.rice.edu/~dougm/twiddle/Hilbert/)
- [Embree](https://www.embree.org/)
- [pbrt-v3](https://github.com/mmp/pbrt-v3)


# Publications

List of publications related to neptun,

<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;">
<span id="w-a">[A]</span> A. Aman, S. Demirci, and U. Güdükbay, “Compact tetrahedralization-based acceleration structures for ray tracing,” Journal of Visualization, vol. 25, no. 5, pp. 1103–1115, Oct. 2022, doi: <a href="https://doi.org/10.1007/s12650-022-00842-x">10.1007/s12650-022-00842-x</a>.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="w-b">[B]</span> A. Aman, S. Demirci, U. Güdükbay, and I. Wald, “Multi-level tetrahedralization-based accelerator for ray-tracing animated scenes,” Computer Animation and Virtual Worlds, vol. 32, no. 3–4, p. e2024, 2021, doi: <a href="https://doi.org/10.1002/cav.2024">https://doi.org/10.1002/cav.2024</a>.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="w-c">[C]</span> Z. Erkoç, A. Aman, U. Güdükbay, and H. Si, “Out-of-core Constrained Delaunay Tetrahedralizations for Large Scenes,” in Numerical Geometry, Grid Generation and Scientific Computing, 2021, pp. 113–124.
</p>

# References

Code contains code from the following works,


<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;">
<span id="r-1">[1]</span> A. Lagae and P. Dutré, “Accelerating ray tracing using constrained tetrahedralizations,” in Computer Graphics Forum, 2008, vol. 27, no. 4, pp. 1303–1312.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="r-2">[2]</span> M. Maria, S. Horna, and L. Aveneau, “Efficient ray traversal of constrained Delaunay tetrahedralization,” in 12th International Joint Conference on Computer Vision, Imaging and Computer Graphics Theory and Applications (VISIGRAPP 2017), 2017, vol. 1, pp. 236–243.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="r-3">[3]</span> M. Pharr, W. Jakob, and G. Humphreys, “Physically Based Rendering: From Theory to Implementation,” 2016.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="r-4">[4]</span> I. Wald, S. Woop, C. Benthin, G. S. Johnson, and M. Ernst, “Embree: a kernel framework for efficient CPU ray tracing,” ACM Transactions on Graphics (TOG), vol. 33, no. 4, pp. 1–8, 2014.
</p>
<p style="padding-left: 1.4em; text-indent: -1.4em;margin-bottom: 0;padding-bottom: 0;margin-top: 0em;padding-top: 0.5em;">
<span id="r-5">[5]</span> S. Hang, “TetGen, a Delaunay-based quality tetrahedral mesh generator,” ACM Trans. Math. Softw, vol. 41, no. 2, p. 11, 2015.
</p>

