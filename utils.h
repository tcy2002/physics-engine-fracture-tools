#pragma once

#include "vector3.h"
#include <vector>

PHYS_NAMESPACE_BEGIN

/**
 * @brief This file provides some data and math utilities for
 * fracture calculation.
 */

#define EPS 0.00001

#ifdef USE_DOUBLE
#define REAL_MAX DBL_MAX
#else
#define REAL_MAX FLT_MAX
#endif

#ifdef USE_DOUBLE
#define REAL_MIN (-DBL_MAX)
#else
#define REAL_MIN (-DBL_MAX)
#endif

FORCE_INLINE bool approx_equal(Real a, Real b) {
    return std::abs(a - b) < EPS;
}

FORCE_INLINE bool approx_equal(const Vector3& a, const Vector3& b) {
    return approx_equal(a.getX(), b.getX()) &&
           approx_equal(a.getY(), b.getY()) &&
           approx_equal(a.getZ(), b.getZ());
}

FORCE_INLINE Vector3 max(const Vector3& a, const Vector3& b) {
    return {std::max(a.getX(), b.getX()), std::max(a.getY(), b.getY()), std::max(a.getZ(), b.getZ())};
}

FORCE_INLINE Vector3 min(const Vector3& a, const Vector3& b) {
    return {std::min(a.getX(), b.getX()), std::min(a.getY(), b.getY()), std::min(a.getZ(), b.getZ())};
}

FORCE_INLINE Vector3 average(const std::vector<Vector3>& list) {
    auto avg = Vector3::Zero();
    for (auto& v : list) {
        avg += v;
    }
    return avg / (Real)list.size();
}

FORCE_INLINE Real calc_mesh_volume(const std::vector<Vector3>& vertices, const std::vector<uint32_t>& indices) {
    Real volume = 0;
    for (int i = 0; i < indices.size(); i += 3) {
        auto& v0 = vertices[indices[i]];
        auto& v1 = vertices[indices[i + 1]];
        auto& v2 = vertices[indices[i + 2]];
        volume += v0.dot(v1.cross(v2)) / 6;
    }
    return volume;
}

FORCE_INLINE void calc_tetrahedron_bounding_sphere(const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4, Vector3& center, Real& radius) {
    auto v1v2 = v2 - v1, v1v3 = v3 - v1, v1v4 = v4 - v1;
    auto v1v2m = (v1 + v2) / 2, v1v3m = (v1 + v3) / 2, v1v4m = (v1 + v4) / 2;
    auto v1v2v3n = cross(v1v2, v1v3), v1v3v4n = cross(v1v3, v1v4);

    auto v1v2n = cross(v1v2, v1v2v3n);
    Real k1 = dot(v1v3m - v1v2m, v1v3) / dot(v1v2n, v1v3);
    auto p1 = v1v2m + v1v2n * k1;

    auto v1v3n = cross(v1v3, v1v3v4n);
    Real k2 = dot(v1v4m - v1v3m, v1v4) / dot(v1v3n, v1v4);
    auto p2 = v1v3m + v1v3n * k2;

    Real k3 = dot(p2 - p1, v1v4) / dot(v1v2v3n, v1v4);
    center = p1 + v1v2v3n * k3;
    radius = (center - v1).norm();
}

FORCE_INLINE bool calc_line_plane_intersection(const Vector3& p, const Vector3& n, const Vector3& v1, const Vector3& v2, Vector3& inter, Real& t) {
    Real d = dot(n, v2 - v1);
    if (approx_equal(d, 0)) {
        return false;
    }

    t = dot(n, p - v1) / d;
    inter = v1 + (v2 - v1) * t;
    return t > EPS && t < 1 - EPS;
}

FORCE_INLINE bool is_point_upside_plane(const Vector3& p, const Vector3& n, const Vector3& v) {
    return dot(v - p, n) > EPS;
}

FORCE_INLINE bool is_point_on_plane(const Vector3& p, const Vector3& n, const Vector3& v) {
    return approx_equal(dot(v - p, n), 0);
}

FORCE_INLINE bool is_point_inside_sphere(const Vector3& p, const Vector3& center, Real radius) {
    return (p - center).norm() < radius - EPS;
}

FORCE_INLINE bool are_points_collinear(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
    return approx_equal(cross(v1 - v2, v3 - v2).norm(), 0);
}

template <typename T>
FORCE_INLINE void sort3(T& v1, T& v2, T& v3) {
    if (v1 > v2) {
        std::swap(v1, v2);
    }
    if (v2 > v3) {
        std::swap(v2, v3);
    }
    if (v1 > v2) {
        std::swap(v1, v2);
    }
}

struct vertex {
    Vector3 pos;
    Vector3 nor;
    vertex() = default;
    vertex(const Vector3& p, const Vector3& n): pos(p), nor(n) {}
};

struct triangle {
    uint32_t vert_ids[3]{};
    triangle() = default;
    triangle(uint32_t v1, uint32_t v2, uint32_t v3) {
        vert_ids[0] = v1;
        vert_ids[1] = v2;
        vert_ids[2] = v3;
    }
};

struct polygon {
    std::vector<uint32_t> vert_ids;
    Vector3 nor;
    polygon() = default;
    explicit polygon(const Vector3& n): nor(n) {}
    void add_vert(uint32_t v, uint32_t idx = UINT32_MAX) {
        if (idx == UINT32_MAX) {
            vert_ids.push_back(v);
        } else {
            vert_ids.insert(vert_ids.begin() + idx, v);
        }
    }
    bool remove_vert(uint32_t idx) {
        if (idx >= vert_ids.size()) {
            return false;
        }
        vert_ids.erase(vert_ids.begin() + idx);
        return true;
    }
    bool operator==(const polygon& p) const {
        return approx_equal(nor, p.nor);
    }
};

struct tetrahedron {
    uint32_t tri_ids[4]{};
    Vector3 center;
    Real radius;
    tetrahedron(): radius(0) {}
    tetrahedron(uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4, const Vector3& c, Real r) : center(c), radius(r) {
        tri_ids[0] = t1;
        tri_ids[1] = t2;
        tri_ids[2] = t3;
        tri_ids[3] = t4;
    }
};

/**
 * @brief only used for I/O between upper layer and fracture process,
 */
struct simple_mesh {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<uint32_t> indices;
    void clear() {
        vertices.clear();
        normals.clear();
        indices.clear();
    }
};

PHYS_NAMESPACE_END

uint32_t hash_func(const physeng::Vector3& v) {
    auto x = (uint32_t)round(v.getX() / EPS);
    auto y = (uint32_t)round(v.getY() / EPS);
    auto z = (uint32_t)round(v.getZ() / EPS);
    return (x * 73856093u) ^ (y * 19349663u) ^ (z * 83492791u);
}

uint32_t hash_func(const physeng::vertex& v) {
    return hash_func(v.pos) ^ (hash_func(v.nor) * 31u);
}

uint32_t hash_func(const physeng::triangle& t) {
    uint32_t v1 = t.vert_ids[0], v2 = t.vert_ids[1], v3 = t.vert_ids[2];
    physeng::sort3(v1, v2, v3);
    return (v1 * 73856093u) ^ (v2 * 19349663u) ^ (v3 * 83492791u);
}

uint32_t hash_func(const physeng::polygon& p) {
    return hash_func(p.nor);
}

uint32_t hash_func(const uint32_t& i) {
    return i;
}

bool equal(const physeng::Vector3& a, const physeng::Vector3& b) {
    return approx_equal(a, b);
}

bool equal(const physeng::vertex& a, const physeng::vertex& b) {
    return approx_equal(a.pos, b.pos) && approx_equal(a.nor, b.nor);
}

bool equal(const physeng::triangle& a, const physeng::triangle& b) {
    return (a.vert_ids[0] == b.vert_ids[0] || a.vert_ids[0] == b.vert_ids[1] || a.vert_ids[0] == b.vert_ids[2]) &&
           (a.vert_ids[1] == b.vert_ids[0] || a.vert_ids[1] == b.vert_ids[1] || a.vert_ids[1] == b.vert_ids[2]) &&
           (a.vert_ids[2] == b.vert_ids[0] || a.vert_ids[2] == b.vert_ids[1] || a.vert_ids[2] == b.vert_ids[2]);
}

bool equal(const physeng::polygon& a, const physeng::polygon& b) {
    return approx_equal(a.nor, b.nor);
}

bool equal(const uint32_t& a, const uint32_t& b) {
    return a == b;
}
