#define USE_DOUBLE

#include <iostream>
#include "utils.h"
#include "hash_vector.h"
#include "test_vector.h"
#include "voronoi_calculator.h"
#include "fracture_calculator.h"
#include "triangle_manager.h"
#include <windows.h>
#include <random>
#include <fstream>
#include <sstream>

void mesh_to_obj(const physeng::simple_mesh& mesh, const std::string& filename) {
    std::ofstream file(filename);
    for (auto& v : mesh.vertices) {
        file << "v " << v.getX() << " " << v.getY() << " " << v.getZ() << std::endl;
    }
    for (auto& n : mesh.normals) {
        file << "vn " << n.getX() << " " << n.getY() << " " << n.getZ() << std::endl;
    }
    uint32_t size = mesh.indices.size();
    for (int i = 0; i < size; i += 3) {
        file << "f " << mesh.indices[i] + 1 << "//" << mesh.indices[i] + 1 << " "
             << mesh.indices[i + 1] + 1 << "//" << mesh.indices[i + 1] + 1 << " "
             << mesh.indices[i + 2] + 1 << "//" << mesh.indices[i + 2] + 1 << std::endl;
    }
    file.close();
}

void obj_to_mesh(const std::string& filename, physeng::simple_mesh& mesh) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string type;
        ss >> type;
        if (type == "v") {
            Real x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.emplace_back(x, y, z);
        } else if (type == "vn") {
            Real x, y, z;
            ss >> x >> y >> z;
            mesh.normals.emplace_back(x, y, z);
        } else if (type == "f") {
            uint32_t v1, v2, v3;
            char tmp;
            ss >> v1 >> tmp >> tmp >> v1 >> v2 >> tmp >> tmp >> v2 >> v3;
            mesh.indices.push_back(v1 - 1);
            mesh.indices.push_back(v2 - 1);
            mesh.indices.push_back(v3 - 1);
        }
    }
    file.close();
}

physeng::Vector3 random_point() {
    static std::default_random_engine e(GetTickCount());
    static std::uniform_real_distribution<Real> d(0.1, 0.9);
    return {d(e), d(e), d(e)};
}

int main()
{
//    // test for hash_Vector
//#define T uint64_t
//    int count = 5000;
//    T limit = 10000;
//    hash_vector<T> h_vec(count * 2);
//    test_vector<T> t_vec;
//    std::default_random_engine e(GetTickCount());
//
//    // test data
//    std::vector<T> to_push_back;
//    for (int i = 0; i < count; i++) {
//        to_push_back.push_back(e() % limit);
//    }
//    std::vector<std::pair<uint32_t , T>> to_insert;
//    for (int i = 0; i < count; i++) {
//        to_insert.emplace_back((e() % count) + i, e() % limit);
//    }
//    std::vector<std::vector<T>> to_append;
//    for (int i = 0; i < count / 10; i++) {
//        to_append.emplace_back();
//        for (int j = 0; j < 10; j++) {
//            to_append.back().push_back(e() % limit);
//        }
//    }
//    std::vector<T> to_search;
//    for (int i = 0; i < count; i++) {
//        to_search.push_back(e() % limit);
//    }
//    std::vector<uint32_t> to_erase;
//    for (int i = 0; i < count; i++) {
//        to_erase.push_back(e() % (count * 3));
//    }
//    std::vector<T> to_remove;
//    for (int i = 0; i < count; i++) {
//        to_remove.push_back(e() % limit);
//    }
//    std::vector<std::pair<uint32_t, T>> to_replace1;
//    for (int i = 0; i < count; i++) {
//        to_replace1.emplace_back(e() % count, e() % limit);
//    }
//
//    // test
//    DWORD start, end;
//    std::vector<bool> h_answers, t_answers;
//
//    h_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_answers[i] = h_vec.push_back(to_push_back[i]);
//    }
//    end = GetTickCount();
//    std::cout << "hash push_back: " << end - start << "ms" << std::endl;
//    t_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_answers[i] = t_vec.push_back(to_push_back[i]);
//    }
//    end = GetTickCount();
//    std::cout << "test push_back: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    auto new_vec = h_vec;
//    h_vec = new_vec;
//
//    h_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_answers[i] = h_vec.insert(to_insert[i].first, to_insert[i].second);
//    }
//    end = GetTickCount();
//    std::cout << "hash insert: " << end - start << "ms" << std::endl;
//    t_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_answers[i] = t_vec.insert(to_insert[i].first, to_insert[i].second);
//    }
//    end = GetTickCount();
//    std::cout << "test insert: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    h_answers.assign(count / 10, false);
//    start = GetTickCount();
//    for (int i = 0; i < count / 10; i++) {
//        h_answers[i] = h_vec.append(to_append[i]);
//    }
//    end = GetTickCount();
//    std::cout << "hash append: " << end - start << "ms" << std::endl;
//    t_answers.assign(count / 10, false);
//    start = GetTickCount();
//    for (int i = 0; i < count / 10; i++) {
//        t_answers[i] = t_vec.append(to_append[i]);
//    }
//    end = GetTickCount();
//    std::cout << "test append: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count / 10; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    h_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_answers[i] = h_vec.contains(to_search[i]);
//    }
//    end = GetTickCount();
//    std::cout << "hash contains: " << end - start << "ms" << std::endl;
//    t_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_answers[i] = t_vec.contains(to_search[i]);
//    }
//    end = GetTickCount();
//    std::cout << "test contains: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_vec.erase(to_erase[i]);
//    }
//    end = GetTickCount();
//    std::cout << "hash erase: " << end - start << "ms" << std::endl;
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_vec.erase(to_erase[i]);
//    }
//    end = GetTickCount();
//    std::cout << "test erase: " << end - start << "ms" << std::endl;
//
//    h_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_answers[i] = h_vec.remove(to_remove[i]);
//    }
//    end = GetTickCount();
//    std::cout << "hash remove: " << end - start << "ms" << std::endl;
//    t_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_answers[i] = t_vec.remove(to_remove[i]);
//    }
//    end = GetTickCount();
//    std::cout << "test remove: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    h_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        h_answers[i] = h_vec.replace(to_replace1[i].first, to_replace1[i].second);
//    }
//    end = GetTickCount();
//    std::cout << "hash replace: " << end - start << "ms" << std::endl;
//    t_answers.assign(count, false);
//    start = GetTickCount();
//    for (int i = 0; i < count; i++) {
//        t_answers[i] = t_vec.replace(to_replace1[i].first, to_replace1[i].second);
//    }
//    end = GetTickCount();
//    std::cout << "test replace: " << end - start << "ms" << std::endl;
//    for (int i = 0; i < count; i++) {
//        ASSERT(h_answers[i] == t_answers[i]);
//    }
//
//    start = GetTickCount();
//    h_vec.clear();
//    end = GetTickCount();
//    std::cout << "hash clear: " << end - start << "ms" << std::endl;
//    start = GetTickCount();
//    t_vec.clear();
//    end = GetTickCount();
//    std::cout << "test clear: " << end - start << "ms" << std::endl;
//    ASSERT(h_vec.size() == t_vec.size() && h_vec.size() == 0);

//    // test for voronoi_calculator
//    physeng::voronoi_calculator calculator;
//    std::vector<physeng::Vector3> points = {
//            {-1, 1, -1},
//            {1, -1, -1},
//            {-1, -1, 1},
//            {1, 1, 1},
//            {-0.5, 0.5, -0.5},
//            {0.5, -0.5, -0.5},
//            {0.5, 0.5, 0.5},
//            {-0.5, -0.5, 0.5},
//    };
//    calculator.triangulate(points);
//    uint32_t size = calculator.point_count();
//    for (uint32_t i = 0; i < size; i++) {
//        auto adj = calculator.get_adjacency(i);
//        std::cout << i << ": ";
//        for (auto& j : adj) {
//            std::cout << j << " ";
//        }
//        std::cout << std::endl;
//    }

    // test for fracture_calculator
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
        points.push_back(random_point());
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
    calculator.fracture(mesh, points.to_vector(), results);
    end = GetTickCount();
    std::cout << "fracture: " << end - start << "ms" << std::endl;
    for (auto& result : results) {
        mesh_to_obj(result, "../mesh" + std::to_string(&result - &results[0]) + ".obj");
    }

    physeng::triangle_manager manager;
    manager.import_from_mesh(results[0]);
    physeng::simple_mesh merge_result;
    manager.export_to_mesh(merge_result);
    mesh_to_obj(merge_result, "../merge_result.obj");

//    // test for import mesh
//    physeng::simple_mesh mesh = {
//            {
//                    {0, 0, 0},
//                    {1, 0, 0},
//                    {2, 0, 0},
//                    {1, 1, 0},
//                    {2, 1, 0},
//                    {1, -1, 0},
//                    {0, 1, 0},
//                    {3, 1, 0}
//            },
//            {
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1},
//                    {0, 0, 1}
//            },
//            {
//                0, 5, 1,
//                0, 1, 3,
//                1, 5, 2,
//                0, 3, 6,
//                1, 2, 4,
//                2, 7, 4,
//                1, 4, 3
//            }
//    };
//    mesh_to_obj(mesh, "../mesh_merge_test.obj");
//    physeng::triangle_manager manager;
//    manager.merge(mesh);
//    std::cout << "vertex: " << manager.vertex_count() << std::endl;
//    std::cout << "triangle: " << manager.triangle_count() << std::endl;
//    std::cout << "face: " << manager.face_count() << std::endl;
//    std::cout << "tetrahedron: " << manager.tetrahedron_count() << std::endl;
//    physeng::simple_mesh result;
//    manager.export_to(result);
//    mesh_to_obj(result, "../mesh_merge_result.obj");

    return 0;
}
