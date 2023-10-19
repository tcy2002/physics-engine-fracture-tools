#pragma once

#include "voronoi_calculator.h"
#include <algorithm>

PHYS_NAMESPACE_BEGIN

/**
 * @brief The core class to calculate the fracture of a mesh.
 *
 * Input: a simple mesh, positions of fragments
 * Output: fragment meshes
 *
 * Current algorithm only support convex mesh, no limitation
 * on other aspects.
 *
 * Calculation process:
 * 1. triangulate the fragment positions into voronoi diagram;
 * 2. read and transform mesh data from triangles into faces;
 * 3. cut the origin mesh by each voronoi cell:
 *   3.1. sequentially cut by each median plane:
 *     3.1.1. cut each face of the mesh;
 *     3.1.2. add a new face of the cutting plane;
 *   3.2. regenerate triangles from faces;
 *   3.3. output the new mesh of the cell.
 * Be aware: input and output mesh is stored in simple_mesh,
 * which need to be transformed in advance.
 */
ATTRIBUTE_ALIGNED16(class) fracture_calculator {
private:
    voronoi_calculator _voronoi{};

    void cut_mesh(const simple_mesh& mesh, std::vector<simple_mesh>& new_meshes) {
        uint32_t point_count = _voronoi.point_count();
        triangle_manager worker;
        worker.import_from_mesh(mesh);

        // generate the new meshes of each point
        for (uint32_t i = 0; i < point_count; i++) {
            triangle_manager result;
            cut_mesh_of_one(worker, i, result);
            new_meshes.push_back({});
            result.export_to_mesh(new_meshes.back());
        }
    }

    void cut_mesh_of_one(const triangle_manager& mesh, uint32_t idx, triangle_manager& new_mesh) {
        auto point = _voronoi.get_point(idx);
        auto adjacent_point_ids = _voronoi.get_adjacency(idx);

        // cut the mesh by each adjacent point
        new_mesh = mesh;
        for (auto other_id : adjacent_point_ids) {
            auto other = _voronoi.get_point(other_id);
            auto center = (point + other) / 2;
            auto normal = (other - point).normalized();
            triangle_manager result;
            cut_mesh_by_plane(new_mesh, center, normal, result);
            new_mesh = result;
        }
    }

    static void cut_mesh_by_plane(triangle_manager& old_mesh, const Vector3& p, const Vector3& n, triangle_manager& new_mesh) {
        uint32_t face_count = old_mesh.face_count();
        hash_vector<Vector3> inter_points(50);

        // cut all the faces
        for (uint32_t face_id = 0; face_id < face_count; face_id++) {
            inter_points.append(cut_face_by_plane(face_id, old_mesh, p, n, new_mesh));
        }
        if (inter_points.size() < 3) {
            return;
        }

        // add a new face of the cutting plane
        polygon new_face(n);
        auto sorted_points = inter_points.to_vector();
        std::sort(sorted_points.begin() + 1, sorted_points.end(), [&](const Vector3& a, const Vector3& b) {
            return dot(cross(a - sorted_points[0], b - sorted_points[0]), n) > 0;
        });
        for (const auto& v : sorted_points) {
            new_face.add_vert(new_mesh.add_vertex(v, n));
        }
        new_mesh.add_face(new_face);
    }

    static std::vector<Vector3> cut_face_by_plane(uint32_t face_id, triangle_manager& old_mesh, const Vector3& p, const Vector3& n, triangle_manager& new_mesh) {
        auto face = old_mesh.get_face(face_id);
        uint32_t vert_count = face.vert_ids.size();
        hash_vector<Vector3> inter_points(vert_count);
        std::vector<vertex> vertices(vert_count);
        std::vector<int> side(vert_count, -1);
        hash_vector<uint32_t> new_point_ids(vert_count * 2);

        // check the side of each vertex to the cutting plane
        for (int i = 0; i < vert_count; i++) {
            vertices[i] = old_mesh.get_vertex(face.vert_ids[i]);
            side[i] = is_point_on_plane(p, n, vertices[i].pos) ? 0 : -1;
            side[i] = is_point_upside_plane(p, n, vertices[i].pos) ? 1 : -1;
        }

        // traverse the segments in order, and add at most 2 intersection points
        for (uint32_t i = 0; i < vert_count; i++) {
            uint32_t j = (i + 1) % vert_count;
            side[i] == 0 && inter_points.push_back(vertices[i].pos);
            side[j] == 0 && inter_points.push_back(vertices[j].pos);

            i == 0 && side[i] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[i].pos, vertices[i].nor));

            Vector3 inter;
            Real t;
            if (side[i] * side[j] < 0) {
                calc_line_plane_intersection(p, n, vertices[i].pos, vertices[j].pos, inter, t);
                auto nor = (vertices[j].nor * t + vertices[i].nor * (1 - t)).normalized();
                new_point_ids.push_back(new_mesh.add_vertex(inter, nor));
                inter_points.push_back(inter);
            }

            j != 0 && side[j] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[j].pos, vertices[j].nor));
        }

        // some corner cases
        if (new_point_ids.size() < 3) {
            return inter_points.to_vector();
        }

        // add the remaining face downside the cutting plane
        polygon new_face;
        auto v1 = new_mesh.get_vertex(new_point_ids[0]).pos;
        auto v2 = new_mesh.get_vertex(new_point_ids[1]).pos;
        auto v3 = new_mesh.get_vertex(new_point_ids[2]).pos;
        new_face.nor = cross(v2 - v1, v3 - v1).normalized();
        new_face.vert_ids = new_point_ids.to_vector();
        new_mesh.add_face(new_face);
        return inter_points.to_vector();
    }

public:
    void fracture(const simple_mesh& mesh, const std::vector<Vector3>& sample_points, std::vector<simple_mesh>& result) {
        _voronoi.triangulate(sample_points);
        cut_mesh(mesh, result);
    }
};

PHYS_NAMESPACE_END
