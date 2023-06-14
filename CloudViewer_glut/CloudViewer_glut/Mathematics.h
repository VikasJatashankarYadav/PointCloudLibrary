/* 
 * This file is an internal part of the Strivision Library.
 * Strivision LLC (C) 2022 
 */
#ifndef PX_MATHEMATICS_H
#define PX_MATHEMATICS_H

#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <fstream>
#include <limits>

#include <Eigen/Eigen>

namespace strivision {

typedef float Real;

static const Real PI = Real(3.14159265);
static const Real HALF_PI = Real(1.57079633);
static const Real QUARTER_PI = Real(0.7853981625);
static const Real E = Real(2.71828182845904523536);

template <typename Real>
using Vector2 = Eigen::Matrix<Real, 2, 1>;

template <typename Real>
using Normal2 = Eigen::Matrix<Real, 2, 1>;

template <typename Real>
using Vector3 = Eigen::Matrix<Real, 3, 1>;

template <typename Real>
using Normal3 = Eigen::Matrix<Real, 3, 1>;

template <typename Real>
using Vector4 = Eigen::Matrix<Real, 4, 1>;

template <typename Real>
using Normal4 = Eigen::Matrix<Real, 4, 1>;

template <typename Real>
using Matrix2 = Eigen::Matrix<Real, 2, 2>;

template <typename Real>
using Matrix3 = Eigen::Matrix<Real, 3, 3>;

template <typename Real>
using Vector = Eigen::Matrix<Real, -1, 1>;

template <typename Real>
using Matrix4 = Eigen::Matrix<Real, 4, 4>;

template <typename Real>
using Matrix = Eigen::Matrix<Real, -1, -1>;

template <typename Real>
using Quaternion = Eigen::Quaternion<Real>;

template <typename Real>
using AngleAxis = Eigen::AngleAxis<Real>;

typedef Vector2<float> Vector2f;
typedef Normal2<float> Normal2f;
typedef Vector3<float> Vector3f;
typedef Normal3<float> Normal3f;
typedef Vector4<float> Vector4f;
typedef Normal4<float> Normal4f;
typedef Matrix2<float> Matrix2f;
typedef Matrix3<float> Matrix3f;
typedef Matrix4<float> Matrix4f;
typedef Vector<float> VectorXf;
typedef Matrix<float> MatrixXf;
typedef Quaternion<float> Quaternionf;

typedef Vector2<double> Vector2d;
typedef Normal2<double> Normal2d;
typedef Vector3<double> Vector3d;
typedef Normal3<double> Normal3d;
typedef Vector4<double> Vector4d;
typedef Normal4<double> Normal4d;
typedef Matrix2<double> Matrix2d;
typedef Matrix3<double> Matrix3d;
typedef Matrix4<double> Matrix4d;
typedef Vector<double> VectorXd;
typedef Matrix<double> MatrixXd;
typedef Quaternion<double> Quaterniond;

typedef Vector2<int> Vector2i;
typedef Vector3<int> Vector3i;
typedef Vector4<int> Vector4i;
typedef Matrix2<int> Matrix2i;
typedef Matrix3<int> Matrix3i;
typedef Matrix4<int> Matrix4i;
typedef Vector<int> VectorXi;
typedef Matrix<int> MatrixXi;

template <typename Real>
inline void ClampToZero(Real& value, Real epsilon) noexcept {
	if ( value > -epsilon && value < epsilon ) value = Real(0);
}

template <typename Real>
inline void ClampToZero(Vector3<Real>& vector, Real epsilon) noexcept {
	ClampToZero(vector.x(), epsilon);
	ClampToZero(vector.y(), epsilon);
	ClampToZero(vector.z(), epsilon);
}

template <typename Real>
Vector3<Real> CartesianToSpherical(const Vector3<Real>& cartesianCoord) {
	Vector3<Real> sphericalCoord;

	Real x = cartesianCoord.x();
	Real z = cartesianCoord.y();
	Real y = cartesianCoord.z();

	Real r = std::sqrt(x*x + y*y + z*z);
	Real theta = std::atan2(y, x);
	Real phi = std::acos(z / r);

	sphericalCoord.x() = r;
	sphericalCoord.y() = theta;
	sphericalCoord.z() = phi;

	return sphericalCoord;
}

template <typename Real>
Vector3<Real> SphericalToCartesian(const Vector3<Real>& sphericalCoord) {
	Vector3<Real> cartesianCoord;
	Real r = sphericalCoord.x();
	Real theta = sphericalCoord.z();
	Real phi = sphericalCoord.y();

	Real cosTheta = std::cos(theta);
	Real cosPhi = std::cos(phi);
	Real sinTheta = std::sin(theta);
	Real sinPhi = std::sin(phi);

	cartesianCoord.x() = r * (cosTheta * sinPhi);  
	cartesianCoord.y() = r * (sinTheta * sinPhi);
	cartesianCoord.z() = r * cosPhi;

	return cartesianCoord;
}

}

#endif
