#define USE_DOUBLE

#include <iostream>
#include <fstream>
#include <windows.h>
#include <random>
#include <utility>
#include "damage/utils.h"
#include "damage/fracture_calculator.h"
#include "Eigen/Core"

#define USE_DOUBLE

void mesh_to_obj(const physeng::simple_mesh& mesh, const std::string& filename) {
    std::ofstream file(filename);
    for (auto& v : mesh.vertices) {
        file << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }
    for (auto& n : mesh.normals) {
        file << "vn " << n.x() << " " << n.y() << " " << n.z() << std::endl;
    }
    uint32_t size = mesh.indices.size();
    for (int i = 0; i < size; i += 3) {
        file << "f " << mesh.indices[i] + 1 << "//" << mesh.indices[i] + 1 << " "
             << mesh.indices[i + 1] + 1 << "//" << mesh.indices[i + 1] + 1 << " "
             << mesh.indices[i + 2] + 1 << "//" << mesh.indices[i + 2] + 1 << std::endl;
    }
    file.close();
}

physeng::Vector3 random_point_3d() {
    static std::default_random_engine e(GetTickCount());
    static std::uniform_real_distribution<Real> d(0.1, 0.9);
    return {d(e), d(e), d(e)};
}

physeng::Vector3 random_point_2d() {
    static std::default_random_engine e(GetTickCount());
    static std::uniform_real_distribution<Real> d(0.1, 0.9);
    return {d(e), d(e), 0};
}


int main()
{
    physeng::fracture_calculator calculator;
    physeng::simple_mesh mesh = {
            {
                    {0, 1, 0},
                    {1, 1, 0},
                    {1, 0, 0},
                    {0, 0, 0},
                    {0, 0, 1},
                    {1, 0, 1},
                    {1, 1, 1},
                    {0, 1, 1},
                    {0, 1, 1},
                    {1, 1, 1},
                    {1, 1, 0},
                    {0, 1, 0},
                    {1, 0, 1},
                    {0, 0, 1},
                    {0, 0, 0},
                    {1, 0, 0},
                    {0, 0, 1},
                    {0, 1, 1},
                    {0, 1, 0},
                    {0, 0, 0},
                    {1, 1, 1},
                    {1, 0, 1},
                    {1, 0, 0},
                    {1, 1, 0},
            },
            {
                    {0, 0, -1},
                    {0, 0, -1},
                    {0, 0, -1},
                    {0, 0, -1},
                    {0, 0, 1},
                    {0, 0, 1},
                    {0, 0, 1},
                    {0, 0, 1},
                    {0, 1, 0},
                    {0, 1, 0},
                    {0, 1, 0},
                    {0, 1, 0},
                    {0, -1, 0},
                    {0, -1, 0},
                    {0, -1, 0},
                    {0, -1, 0},
                    {-1, 0, 0},
                    {-1, 0, 0},
                    {-1, 0, 0},
                    {-1, 0, 0},
                    {1, 0, 0},
                    {1, 0, 0},
                    {1, 0, 0},
                    {1, 0, 0},
            },
            {
                0, 1, 2,
                0, 2, 3,
                4, 5, 6,
                4, 6, 7,
                8, 9, 10,
                8, 10, 11,
                12, 13, 14,
                12, 14, 15,
                16, 17, 18,
                16, 18, 19,
                20, 21, 22,
                20, 22, 23,
            }
    };

    hash_vector<physeng::Vector3> points(100);
    for (int i = 0; i < 50; i++) {
        points.push_back(random_point_3d());
    }
//    std::vector<physeng::Vector3> points = {
//            {0.2, 0.2, 0.6},
//            {0.8, 0.2, 0.5},
//            {0.5, 0.8, 0.4}
//    };

    mesh_to_obj(mesh, "../cube.obj");
    std::vector<physeng::simple_mesh> results;
    DWORD start, end;
    start = GetTickCount();
    calculator.triangulate(points.to_vector());
    calculator.fracture(mesh, results);
    end = GetTickCount();
    std::cout << "fracture: " << end - start << "ms" << std::endl;
    for (auto& result : results) {
        mesh_to_obj(result, "../mesh" + std::to_string(&result - &results[0]) + ".obj");
    }

    return 0;
}
