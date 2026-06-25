// Standalone binary wrapping the DACPO-modified PoissonRecon.
// Reads oriented point cloud (+ optional per-vertex weights) from PLY,
// runs the DACPO-modified Poisson reconstruction, writes mesh PLY.
//
// Usage: dacpo_psr --in input.ply --out output.ply [PoissonRecon flags...]
//
// The input PLY must have vertex properties: x y z nx ny nz
// Optionally: a "weight" double property per vertex (from sample_points).

#include "PoissonRecon.h"
#include "happly.h"
#include <string>
#include <vector>
#include <cstdio>

int main(int argc, char* argv[]) {
    std::string in_path, out_path;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--in" && i + 1 < argc) { in_path = argv[i + 1]; }
        else if (arg == "--out" && i + 1 < argc) { out_path = argv[i + 1]; }
    }

    if (in_path.empty() || out_path.empty()) {
        fprintf(stderr, "Usage: dacpo_psr --in <input.ply> --out <output.ply> [PoissonRecon flags...]\n");
        return 1;
    }

    happly::PLYData ply_in(in_path);
    auto& verts = ply_in.getElement("vertex");

    std::vector<double> x = verts.getProperty<double>("x");
    std::vector<double> y = verts.getProperty<double>("y");
    std::vector<double> z = verts.getProperty<double>("z");
    std::vector<double> nx_v = verts.getProperty<double>("nx");
    std::vector<double> ny_v = verts.getProperty<double>("ny");
    std::vector<double> nz_v = verts.getProperty<double>("nz");

    std::vector<double> weights;
    if (verts.hasProperty("weight")) {
        weights = verts.getProperty<double>("weight");
    }

    size_t n = x.size();
    std::vector<std::pair<Point<double, 3>, Normal<double, 3>>> points_normals(n);
    for (size_t i = 0; i < n; i++) {
        points_normals[i].first[0] = x[i];
        points_normals[i].first[1] = y[i];
        points_normals[i].first[2] = z[i];
        points_normals[i].second.normal[0] = nx_v[i];
        points_normals[i].second.normal[1] = ny_v[i];
        points_normals[i].second.normal[2] = nz_v[i];
    }

    auto mesh = poisson_reconstruction<double, 3>(
        argc, argv, points_normals,
        weights.empty() ? nullptr : &weights);

    if (mesh.first.empty()) {
        fprintf(stderr, "dacpo_psr: reconstruction produced no vertices\n");
        _Exit(1);
    }

    size_t nv = mesh.first.size();
    std::vector<float> mx(nv), my(nv), mz(nv);
    for (size_t i = 0; i < nv; i++) {
        mx[i] = (float)mesh.first[i][0];
        my[i] = (float)mesh.first[i][1];
        mz[i] = (float)mesh.first[i][2];
    }

    std::vector<std::vector<int>> faces_out;
    faces_out.reserve(mesh.second.size());
    for (auto& f : mesh.second) {
        faces_out.push_back(f);
    }

    happly::PLYData ply_out;
    ply_out.addElement("vertex", nv);
    ply_out.getElement("vertex").addProperty<float>("x", mx);
    ply_out.getElement("vertex").addProperty<float>("y", my);
    ply_out.getElement("vertex").addProperty<float>("z", mz);
    ply_out.addElement("face", faces_out.size());
    ply_out.getElement("face").addListProperty<int>("vertex_indices", faces_out);
    ply_out.write(out_path, happly::DataFormat::Binary);

    _Exit(0);
}
