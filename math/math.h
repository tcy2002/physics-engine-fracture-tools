

#include "common/general.h"
#include "common/real.h"

#define PE_USE_BACKEND_EIGEN 

#ifdef PE_USE_BACKEND_EIGEN

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCore"
#include "Eigen/Geometry"

PHYS_NAMESPACE_BEGIN

#ifdef USE_DOUBLE

using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3x3 = Eigen::Matrix3d;
using Matrix4x4 = Eigen::Matrix<double, 4, 4>;
using Quaternion = Eigen::Quaterniond;

#else

using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;
using Matrix3x3 = Eigen::Matrix3f;
using Matrix4x4 = Eigen::Matrix<float, 4, 4>;
using Quaternion = Eigen::Quaternionf;

#endif


PHYS_NAMESPACE_END

#elif PE_USE_BACKEND_WRAPPER

#include <linear_math/vector.h>
#include <linear_math/matrix.h>
#include <linear_math/quaternion.h>

PHYS_NAMESPACE_BEGIN

using Vector3 = linear_math::Vector3;
using Matrix3x3 = linear_math::Matrix3;
using Matrix4x4 = linear_math::Matrix4;
using Quaternion = linear_math::Quaternion;

PHYS_NAMESPACE_END

#else

#include "math/matrix3x3.h"
#include "math/quaternion.h"

#endif

