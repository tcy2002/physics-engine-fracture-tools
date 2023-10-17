#pragma once

#include "general.h"
#include "voronoi_calculator.h"
#include <map>
#include <algorithm>

PHYS_NAMESPACE_BEGIN

ATTRIBUTE_ALIGNED16(class) fracture_calculator {
private:
    voronoi_calculator _voronoi{};

    void cut_mesh(const simple_mesh& mesh, std::vector<simple_mesh>& new_meshes) {
        uint32_t point_count = _voronoi.point_count();
        triangle_manager worker(mesh);

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

        new_mesh = mesh;
        for (auto other_id : adjacent_point_ids) {
            auto other = _voronoi.get_point(other_id);
            auto center = (point + other) / 2;
            auto normal = (other - point).normalized();
            triangle_manager result;
            cut_mesh_by_plane(new_mesh, center, normal, result);
            new_mesh = result;
        }

        new_mesh.to_triangles();
    }

    static void cut_mesh_by_plane(triangle_manager& old_mesh, const Vector3& p, const Vector3& n, triangle_manager& new_mesh) {
        uint32_t face_count = old_mesh.face_count();
        hash_vector<Vector3> inter_points(50);

        // cut all the faces
        for (uint32_t face_id = 0; face_id < face_count; face_id++) {
            inter_points.append(cut_face_by_plane(face_id, old_mesh, p, n, new_mesh));
        }

        // add the new face of the cutting plane
        if (inter_points.empty()) {
            return;
        }
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
        std::vector<uint32_t> new_point_ids;
        std::vector<Vector3> inter_points;
        auto face = old_mesh.get_face(face_id);
        uint32_t vert_count = face.vert_ids.size();

        for (int i = 0; i < vert_count; i++) {
            auto v1 = old_mesh.get_vertex(face.vert_ids[i]);
            auto v2 = old_mesh.get_vertex(face.vert_ids[(i + 1) % vert_count]);

            if (i == 0 && !is_point_upside_plane(p, n, v1.pos)) {
                new_point_ids.push_back(new_mesh.add_vertex(v1.pos, v1.nor));
                std::cout << new_point_ids.back() << " " << v1.pos << " ";
            }

            Vector3 inter;
            Real t;
            if (calc_line_plane_intersection(p, n, v1.pos, v2.pos, inter, t)) {
                auto nor = (v2.nor * t + v1.nor * (1 - t)).normalized();
                new_point_ids.push_back(new_mesh.add_vertex(inter, nor));
                std::cout << new_point_ids.back() << " " << inter << " ";
                inter_points.push_back(inter);
            }

            if (i != vert_count - 1 && !is_point_upside_plane(p, n, v2.pos)) {
                new_point_ids.push_back(new_mesh.add_vertex(v2.pos, v2.nor));
                std::cout << new_point_ids.back() << " " << v2.pos << " ";
            }
        }
        std::cout << std::endl;

        if (new_point_ids.empty()) {
            return {};
        }
        polygon new_face;
        auto v1 = new_mesh.get_vertex(new_point_ids[0]).pos;
        auto v2 = new_mesh.get_vertex(new_point_ids[1]).pos;
        auto v3 = new_mesh.get_vertex(new_point_ids[2]).pos;
        new_face.nor = cross(v2 - v1, v3 - v1);
        if (new_face.nor.fuzzyZero()) {
            // TODO: ???
            std::cout << "bomb!" << std::endl;
            return inter_points;
        } else {
            new_face.nor.normalize();
        }
        new_face.vert_ids = new_point_ids;
        new_mesh.add_face(new_face);
        return inter_points;
    }

public:
    void fracture(const simple_mesh& mesh, const std::vector<Vector3>& sample_points, std::vector<simple_mesh>& result) {
        _voronoi.triangulate(sample_points);
        cut_mesh(mesh, result);
    }
};

PHYS_NAMESPACE_END