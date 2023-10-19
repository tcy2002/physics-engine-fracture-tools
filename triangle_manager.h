#pragma once

#include "utils.h"
#include "hash_vector.h"

PHYS_NAMESPACE_BEGIN

#define VERTEX_CAPACITY 997
#define TRIANGLE_CAPACITY 997
#define FACE_CAPACITY 997

/**
 * @brief The mesh manager class, which is used to manage the
 * mesh data, including vertices, triangles, faces(2d polygons)
 * and tetrahedrons.
 *
 * Data of vertices, triangles and faces are stored in hash_vector,
 * since they need both sequence information and random access
 * efficiency.
 *
 * Only support convex mesh, no limitation on other aspects.
 *
 * A general mesh has some features:
 * - it's composed of polygonal faces, and each face consists of
 *   several triangles;
 * - each vertices has its own normal, but all the triangles of a
 *   face share the same normal;
 * - each vertices of a face must be in the same plane, and the
 *   positions must differ, but vertices of different faces may be
 *   the same position, as long as they have different normals.
 * This structure can automatically cater to these features, and
 * provide some efficiency for the upper layer.
 */
ATTRIBUTE_ALIGNED16(class) triangle_manager {
private:
    hash_vector<vertex> _vertices;
    hash_vector<triangle> _triangles;
    hash_vector<polygon> _faces;
    std::vector<tetrahedron> _tetrahedrons;

    void to_triangles() {
        // transform faces into triangles
        clear_triangles();
        uint32_t face_count = _faces.size();
        for (int i = 0; i < face_count; i++) {
            uint32_t vert_count = _faces[i].vert_ids.size();
            for (int j = 1; j < vert_count - 1; j++) {
                add_triangle(_faces[i].vert_ids[0], _faces[i].vert_ids[j], _faces[i].vert_ids[j + 1]);
            }
        }
    }

public:
    triangle_manager():
    _vertices(VERTEX_CAPACITY),
    _triangles(TRIANGLE_CAPACITY),
    _faces(FACE_CAPACITY) {}

    void clear() {
        _vertices.clear();
        _triangles.clear();
        _faces.clear();
        _tetrahedrons.clear();
    }

    uint32_t vertex_count() const {
        return _vertices.size();
    }

    uint32_t add_vertex(const Vector3& p, const Vector3& n = Vector3::Up()) {
        vertex new_vertex(p, n);
        uint32_t idx = _vertices.index_of(new_vertex);
        if (idx == NOT_FOUND) {
            _vertices.push_back(new_vertex);
            return _vertices.size() - 1;
        }
        return idx;
    }

    vertex get_vertex(uint32_t idx) {
        return _vertices[idx];
    }

    void remove_vertex(uint32_t idx) {
        _vertices.erase(idx);
        hash_vector<triangle> new_triangles(TRIANGLE_CAPACITY);
        for (auto tri : _triangles) {
            for (auto& vert_id : tri.vert_ids) {
                if (vert_id == idx) {
                    vert_id = NOT_FOUND;
                } else if (vert_id > idx) {
                    vert_id--;
                }
            }
            new_triangles.push_back(tri);
        }
        _triangles = new_triangles;
    }

    void clear_vertices() {
        _vertices.clear();
    }

    uint32_t triangle_count() const {
        return _triangles.size();
    }

    uint32_t add_triangle(uint32_t v1, uint32_t v2, uint32_t v3) {
        triangle new_tri(v1, v2, v3);
        uint32_t idx = _triangles.index_of(new_tri);
        if (idx == NOT_FOUND) {
            _triangles.push_back(new_tri);
            return _triangles.size() - 1;
        }
        return idx;
    }

    triangle get_triangle(uint32_t idx) {
        return _triangles[idx];
    }

    void remove_triangle(uint32_t idx) {
        _triangles.erase(idx);
        for (auto& tet : _tetrahedrons) {
            for (auto& tri_id : tet.tri_ids) {
                if (tri_id == idx) {
                    tri_id = NOT_FOUND;
                } else if (tri_id > idx) {
                    tri_id--;
                }
            }
        }
    }

    void clear_triangles() {
        _triangles.clear();
    }

    uint32_t face_count() const {
        return _faces.size();
    }

    void add_face(const polygon& face) {
        _faces.push_back(face);
    }

    polygon get_face(uint32_t idx) {
        return _faces[idx];
    }

    void remove_face(uint32_t idx) {
        _faces.erase(idx);
    }

    void clear_faces() {
        _faces.clear();
    }

    uint32_t tetrahedron_count() const {
        return _tetrahedrons.size();
    }

    void add_tetrahedron(uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4,
                         uint32_t p1, uint32_t p2, uint32_t p3, uint32_t p4) {
        Vector3 center;
        Real radius;
        calc_tetrahedron_bounding_sphere(_vertices[p1].pos, _vertices[p2].pos,
                                         _vertices[p3].pos, _vertices[p4].pos,
                                         center, radius);
        _tetrahedrons.emplace_back(t1, t2, t3, t4, center, radius);
    }

    tetrahedron get_tetrahedron(uint32_t idx) const {
        return _tetrahedrons[idx];
    }

    void remove_tetrahedron(uint32_t idx) {
        _tetrahedrons.erase(_tetrahedrons.begin() + idx);
    }

    void clear_tetrahedrons() {
        _tetrahedrons.clear();
    }

    bool add_triangle_to_face(uint32_t v1i, uint32_t v2i, uint32_t v3i) {
        auto v1p = _vertices[v1i].pos;
        auto v2p = _vertices[v2i].pos;
        auto v3p = _vertices[v3i].pos;
        auto n = cross(v2p - v1p, v3p - v1p).normalized();
        uint32_t vs[] = {v1i, v2i, v3i};

        // use normal to identify a face
        polygon new_face(n);
        uint32_t face_id = _faces.index_of(new_face);

        // if the face doesn't exist, create a new one
        if (face_id == NOT_FOUND) {
            for (auto v : vs) {
                new_face.add_vert(v);
            }
            _faces.push_back(new_face);
            return true;
        }

        // check if a new vertex needs to be added or removed
        uint32_t count = _faces[face_id].vert_ids.size();
        for (int i = 0; i < count; i++) {
            uint32_t u1 = _faces[face_id].vert_ids[i],
                     u2 = _faces[face_id].vert_ids[(i + 1) % count],
                     u3 = _faces[face_id].vert_ids[(i + 2) % count];
            // already contains all 3 vertices: need to remove
            // note: only remove the index, not actually remove the vertex,
            // which will not affect the correctness of the mesh structure.
            // removing vertex costs a lot of time, and should be avoided.
            if ((u1 == v1i || u2 == v2i || u3 == v3i) &&
                (u1 == v2i || u2 == v3i || u3 == v1i) &&
                (u1 == v3i || u2 == v1i || u3 == v2i)) {
                _faces[face_id].remove_vert((i + 1) % count);
                return true;
            }

            // only contains 2 vertices: need to add
            for (int j = 0; j < 3; j++) {
                uint32_t v1 = vs[j], v2 = vs[(j + 1) % 3];
                if ((u1 == v1 && u2 == v2) || (u1 == v2 && u2 == v1)) {
                    _faces[face_id].add_vert(vs[(j + 2) % 3], (i + 1) % count);
                    return true;
                }
            }
        }

        // this means the triangle is separated from the face, should be added
        // later. (until this triangle is right close to the border)
        return false;
    }

    void import_from_mesh(const simple_mesh& mesh) {
        // import data from a new mesh, automatically generate faces, simplify
        // the mesh structure and merge into current mesh.
        clear();
        ASSERT(mesh.vertices.size() == mesh.normals.size() && mesh.indices.size() % 3 == 0);

        // add vertices
        uint32_t vert_count = mesh.vertices.size();
        std::vector<uint32_t> vert_ids(vert_count);
        for (uint32_t i = 0; i < vert_count; i++) {
            vert_ids[i] = add_vertex(mesh.vertices[i], mesh.normals[i]);
        }

        // add faces: need to re-add until all triangles are added to face
        uint32_t tri_count = mesh.indices.size() / 3;
        std::vector<bool> tri_added(tri_count, false);
        int num = 0;
        while (num != tri_count) {
            for (uint32_t i = 0; i < tri_count; i++) {
                uint32_t base = i * 3;
                !tri_added[i] && (tri_added[i] = add_triangle_to_face(
                        mesh.indices[base], mesh.indices[base + 1], mesh.indices[base + 2])) && ++num;
            }
        }
    }

    void export_to_mesh(simple_mesh& mesh) {
        // export the mesh data into a mesh
        mesh.clear();
        to_triangles();
        uint32_t vert_size = _vertices.size(), tri_size = _triangles.size();

        mesh.vertices.resize(vert_size);
        mesh.normals.resize(vert_size);
        mesh.indices.resize(tri_size * 3);

        // export vertices
        for (uint32_t i = 0; i < vert_size; i++) {
            mesh.vertices[i] = _vertices[i].pos;
            mesh.normals[i] = _vertices[i].nor;
        }

        // export triangles
        for (uint32_t i = 0; i < tri_size; i++) {
            uint32_t base = i * 3;
            mesh.indices[base] = _triangles[i].vert_ids[0];
            mesh.indices[base + 1] = _triangles[i].vert_ids[1];
            mesh.indices[base + 2] = _triangles[i].vert_ids[2];
        }
    }
};

PHYS_NAMESPACE_END
