#pragma once
#include "PointStream.h"
#include "PointStreamData.h"
#include "happly.h"

#include <spawn.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <sstream>
#include <thread>
#include <atomic>
extern char **environ;

#define MESH std::pair<std::vector<Point<REAL, DIM>>,std::vector<std::vector<int>>>
#define MESH_D std::pair<std::vector<Point<double, 3>>,std::vector<std::vector<int>>>

#define POINTS_NORMALS std::vector<std::pair<Point<REAL, DIM>, Normal<REAL, (int)DIM>>>
#define POINTS_NORMALS_D std::vector<std::pair<Point<double, 3>, Normal<double, 3>>>

#define ORIENTED_POINTS std::vector<std::pair<Point<REAL, DIM>, Normal<REAL, (int)DIM>>>

#ifndef POISSONRECON_BINARY
#define POISSONRECON_BINARY "dacpo_psr"
#endif

POINTS_NORMALS_D sample_points_d(int argc, char* argv[], const POINTS_NORMALS_D& points_normals, XForm<double, 3 + 1>& iXForm, std::vector<double>* weight_samples);
MESH_D poisson_reconstruction_d(int argc, char* argv[], const POINTS_NORMALS_D& points_normals, const std::vector<double>* weight_samples);

#ifndef PSR_USE_SUBPROCESS
#define PSR_USE_SUBPROCESS 1
#endif

namespace poisson_subprocess {

inline std::string temp_dir() {
    auto p = std::filesystem::temp_directory_path() / "dacpo_psr";
    std::filesystem::create_directories(p);
    return p.string();
}

inline std::string unique_prefix() {
    static std::atomic<int> counter{0};
    std::ostringstream ss;
    ss << std::this_thread::get_id() << "_" << counter.fetch_add(1);
    return ss.str();
}

template <class Real, unsigned int Dim>
void write_oriented_ply(
    const std::string& path,
    const std::vector<std::pair<Point<Real, Dim>, Normal<Real, (int)Dim>>>& points_normals,
    const std::vector<double>* weight_samples)
{
    size_t n = points_normals.size();
    std::vector<double> x(n), y(n), z(n), nx(n), ny(n), nz(n);
    for (size_t i = 0; i < n; i++) {
        x[i] = points_normals[i].first[0];
        y[i] = points_normals[i].first[1];
        z[i] = points_normals[i].first[2];
        nx[i] = points_normals[i].second.normal[0];
        ny[i] = points_normals[i].second.normal[1];
        nz[i] = points_normals[i].second.normal[2];
    }
    happly::PLYData ply;
    ply.addElement("vertex", n);
    ply.getElement("vertex").addProperty<double>("x", x);
    ply.getElement("vertex").addProperty<double>("y", y);
    ply.getElement("vertex").addProperty<double>("z", z);
    ply.getElement("vertex").addProperty<double>("nx", nx);
    ply.getElement("vertex").addProperty<double>("ny", ny);
    ply.getElement("vertex").addProperty<double>("nz", nz);
    if (weight_samples && weight_samples->size() == n) {
        ply.getElement("vertex").addProperty<double>("weight", *weight_samples);
    }
    ply.write(path, happly::DataFormat::Binary);
}

template <class Real, unsigned int Dim>
std::pair<std::vector<Point<Real, Dim>>, std::vector<std::vector<int>>>
read_mesh_ply(const std::string& path)
{
    happly::PLYData ply(path);

    std::vector<double> x = ply.getElement("vertex").getProperty<double>("x");
    std::vector<double> y = ply.getElement("vertex").getProperty<double>("y");
    std::vector<double> z = ply.getElement("vertex").getProperty<double>("z");

    std::vector<Point<Real, Dim>> vertices(x.size());
    for (size_t i = 0; i < x.size(); i++) {
        vertices[i][0] = (Real)x[i];
        vertices[i][1] = (Real)y[i];
        vertices[i][2] = (Real)z[i];
    }

    std::vector<std::vector<int>> faces;
    auto& face_elem = ply.getElement("face");
    std::string prop_name = face_elem.hasProperty("vertex_indices") ? "vertex_indices" : "vertex_index";
    auto face_indices = face_elem.getListProperty<int>(prop_name);
    faces.reserve(face_indices.size());
    for (auto& f : face_indices) {
        faces.push_back(std::move(f));
    }

    return {std::move(vertices), std::move(faces)};
}

inline std::vector<std::string> build_args(int argc, char* argv[],
                                            const std::string& in_path,
                                            const std::string& out_path)
{
    std::vector<std::string> args;
    args.push_back(POISSONRECON_BINARY);

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--in") { i++; continue; }
        if (arg == "--out" || arg == "--myout") { i++; continue; }
        args.push_back(arg);
    }
    args.push_back("--in");  args.push_back(in_path);
    args.push_back("--out"); args.push_back(out_path);
    return args;
}

inline int run_process(const std::vector<std::string>& args) {
    std::ostringstream cmd;
    cmd << "OMP_NUM_THREADS=1 KMP_DUPLICATE_LIB_OK=TRUE";
    for (auto& a : args) {
        cmd << " '";
        for (char c : a) { if (c == '\'') cmd << "'\\''"; else cmd << c; }
        cmd << "'";
    }
    cmd << " >/dev/null";

    std::string cmd_str = cmd.str();
    pid_t pid;
    const char* shell_argv[] = {"/bin/sh", "-c", cmd_str.c_str(), nullptr};
    int status = posix_spawn(&pid, "/bin/sh", nullptr, nullptr,
                             const_cast<char**>(shell_argv), environ);

    if (status != 0) { fprintf(stderr, "[PSR] spawn failed: %s\n", strerror(status)); return -1; }
    waitpid(pid, &status, 0);
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) { fprintf(stderr, "[PSR] killed by signal %d\n", WTERMSIG(status)); return -1; }
    return -1;
}

} // namespace poisson_subprocess


// sample_points stays in-process (called once during init, needs correct weights/ixform)
template <class REAL, unsigned int DIM>
POINTS_NORMALS sample_points_entrance(int argc, char *argv[], const POINTS_NORMALS &points_normals, XForm<REAL, DIM + 1> &iXForm, std::vector<double> *weight_samples) {
    int prev_threads = omp_get_max_threads();
    auto result = sample_points_d(argc, argv, points_normals, iXForm, weight_samples);
    omp_set_num_threads(prev_threads);
    return result;
}

template <class REAL, unsigned int DIM>
MESH poisson_reconstruction_entrance(int argc, char *argv[], const POINTS_NORMALS &points_normals, const std::vector<double> *weight_samples) {
#if PSR_USE_SUBPROCESS
    std::string prefix = poisson_subprocess::temp_dir() + "/" + poisson_subprocess::unique_prefix();
    std::string in_path = prefix + "_in.ply";
    std::string out_path = prefix + "_out.ply";

    poisson_subprocess::write_oriented_ply<REAL, DIM>(in_path, points_normals, weight_samples);

    auto args = poisson_subprocess::build_args(argc, argv, in_path, out_path);
    int ret = poisson_subprocess::run_process(args);

    MESH mesh;
    if (ret == 0 && std::filesystem::exists(out_path)) {
        mesh = poisson_subprocess::read_mesh_ply<REAL, DIM>(out_path);
    } else {
        fprintf(stderr, "[ERROR] PoissonRecon subprocess failed (exit %d)\n", ret);
    }

    std::filesystem::remove(in_path);
    std::filesystem::remove(out_path);

    return mesh;
#else
    int prev_threads = omp_get_max_threads();
    auto result = poisson_reconstruction_d(argc, argv, points_normals, weight_samples);
    omp_set_num_threads(prev_threads);
    return result;
#endif
}
