#pragma once

#include <vector>
#include <algorithm>
#include "triangle_manager.h"
#include "utils.h"

PHYS_NAMESPACE_BEGIN

/**
 * @brief A class to calculate the 3d delaunay triangulation
 * of a set of points.
 *
 * Based on the Bowyer-Watson algorithm.
 * process:
 * 1. add a bounding box to the diagram;
 * 2. sequentially add each point to the diagram:
 *   2.1. find all tetrahedrons that don't cater to the Delaunay condition;
 *   2.2. find all triangles that are supposed to be removed;
 *   2.3. add the new point, and corresponding triangles and tetrahedrons;
 * 3. remove the bounding box.
 */
ATTRIBUTE_ALIGNED16(class) voronoi_calculator {
private:
    triangle_manager _manager{};
    std::vector<hash_vector<uint32_t>> adjacency_list{};

    void add_bounding_box(const std::vector<Vector3>& points) {
        // find aabb bounds
        auto min = Vector3::One() * REAL_MAX;
        auto max = Vector3::One() * REAL_MIN;
        for (auto& p : points) {
            min = physeng::min(min, p);
            max = physeng::max(max, p);
        }

        // dilate to include all possible points
        auto center = (min + max) / 2;
        min = center + (min - center) * 3 - Vector3::One() * 0.1;
        max = center + (max - center) * 4 + Vector3::One() * 0.1;

        // add bounding box points
        uint32_t v1i = _manager.add_vertex(min);
        uint32_t v2i = _manager.add_vertex({max.getX(), max.getY(), min.getZ()});
        uint32_t v3i = _manager.add_vertex({max.getX(), min.getY(), max.getZ()});
        uint32_t v4i = _manager.add_vertex({min.getX(), max.getY(), max.getZ()});

        // add bounding box triangles
        uint32_t t1i = _manager.add_triangle(v1i, v2i, v3i);
        uint32_t t2i = _manager.add_triangle(v2i, v1i, v4i);
        uint32_t t3i = _manager.add_triangle(v3i, v2i, v4i);
        uint32_t t4i = _manager.add_triangle(v3i, v4i, v1i);

        // add bounding box tetrahedrons
        _manager.add_tetrahedron(t1i, t2i, t3i, t4i, v1i, v2i, v3i, v4i);
    }

    void remove_bounding_box() {
        // clear the tetrahedrons
        _manager.clear_tetrahedrons();

        // remove triangles that contain bounding box points
        for (uint32_t i = _manager.triangle_count() - 1; i != NOT_FOUND; i--) {
            auto tri = _manager.get_triangle(i);
            if (tri.vert_ids[0] < 4 || tri.vert_ids[1] < 4 || tri.vert_ids[2] < 4) {
                _manager.remove_triangle(i);
            }
        }

        // remove bounding box points
        for (uint32_t i = 0; i < 4; i++) {
            _manager.remove_vertex(0);
        }
    }

    void generate(const std::vector<Vector3>& points) {
        // generate triangle mesh for all points, using Bowyer-Watson algorithm
        for (auto& point : points) {
            // find all tetrahedrons that don't cater to the Delaunay condition
            std::vector<uint32_t> bad_tetrahedrons;
            uint32_t tet_count = _manager.tetrahedron_count();
            for (uint32_t i = 0; i < tet_count; i++) {
                auto tet = _manager.get_tetrahedron(i);
                if (is_point_inside_sphere(point, tet.center, tet.radius)) {
                    bad_tetrahedrons.push_back(i);
                }
            }

            // find all triangles that are part of the boundary of the cavity
            // or should be removed
            hash_vector<uint32_t> good_triangles(_manager.triangle_count() * 2);
            std::vector<uint32_t> bad_triangles;
            for (auto tet_id : bad_tetrahedrons) {
                auto tet = _manager.get_tetrahedron(tet_id);
                for (auto tri_id : tet.tri_ids) {
                    if (good_triangles.contains(tri_id)) {
                        good_triangles.remove(tri_id);
                        bad_triangles.push_back(tri_id);
                    } else {
                        good_triangles.push_back(tri_id);
                    }
                }
            }

            // special case: if the point is on the plane of a triangle
            // then the triangle should be removed
            for (uint32_t i = good_triangles.size() - 1; i != NOT_FOUND; i--) {
                auto tri = _manager.get_triangle(good_triangles[i]);
                auto v1 = _manager.get_vertex(tri.vert_ids[0]).pos;
                auto v2 = _manager.get_vertex(tri.vert_ids[1]).pos;
                auto v3 = _manager.get_vertex(tri.vert_ids[2]).pos;
                if (is_point_on_plane(point, v1, cross(v2 - v1, v3 - v1))) {
                    bad_triangles.push_back(good_triangles[i]);
                    good_triangles.erase(i);
                }
            }

            // add new vertices, triangles and tetrahedrons
            auto vi = _manager.add_vertex(point);
            for (auto tri_id : good_triangles) {
                auto tri = _manager.get_triangle(tri_id);
                auto tri_id1 = _manager.add_triangle(vi, tri.vert_ids[0], tri.vert_ids[1]);
                auto tri_id2 = _manager.add_triangle(vi, tri.vert_ids[1], tri.vert_ids[2]);
                auto tri_id3 = _manager.add_triangle(vi, tri.vert_ids[2], tri.vert_ids[0]);
                _manager.add_tetrahedron(tri_id, tri_id1,tri_id2, tri_id3,
                                         vi, tri.vert_ids[0], tri.vert_ids[1], tri.vert_ids[2]);
            }

            // remove bad tetrahedrons
            for (uint32_t i = bad_tetrahedrons.size() - 1; i != NOT_FOUND; i--) {
                _manager.remove_tetrahedron(bad_tetrahedrons[i]);
            }

            // remove bad triangles
            std::sort(bad_triangles.begin(), bad_triangles.end());
            for (uint32_t i = bad_triangles.size() - 1; i != NOT_FOUND; i--) {
                _manager.remove_triangle(bad_triangles[i]);
            }
        }
    }

    void get_adjacency_list() {
        // get adjacency list according to triangles
        uint32_t vert_count = _manager.vertex_count();
        if (vert_count == 0) {
            return;
        }

        adjacency_list.clear();
        for (uint32_t i = 0; i < vert_count; i++) {
            adjacency_list.emplace_back(vert_count * 2);
        }

        uint32_t tri_count = _manager.triangle_count();
        for (uint32_t i = 0; i < tri_count; i++) {
            auto t = _manager.get_triangle(i);
            adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[1]);
            adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[2]);
            adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[0]);
            adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[2]);
            adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[0]);
            adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[1]);
        }
    }

public:
    void triangulate(const std::vector<Vector3>& points) {
        _manager.clear();
        add_bounding_box(points);
        generate(points);
        remove_bounding_box();
        get_adjacency_list();
    }

    uint32_t point_count() const {
        return _manager.vertex_count();
    }

    Vector3 get_point(uint32_t vert_id) {
        return _manager.get_vertex(vert_id).pos;
    }

    std::vector<uint32_t> get_adjacency(uint32_t vert_id) const {
        return adjacency_list[vert_id].to_vector();
    }
};

PHYS_NAMESPACE_END
