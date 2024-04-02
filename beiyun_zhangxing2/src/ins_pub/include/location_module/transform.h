#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
namespace navicore
{

   template <typename FloatType>
	class Rigid3 {
	 public:
	  typedef  Eigen::Transform<FloatType, 3, Eigen::Affine> Affine;
	  typedef  Eigen::Matrix<FloatType, 3, 1> Vector;
	  typedef  Eigen::Quaternion<FloatType> Quaternion;
	  typedef  Eigen::AngleAxis<FloatType> AngleAxis;

	  Rigid3()
		  : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
	  // TODO(damonkohler): Remove
	  explicit Rigid3(const Affine& affine)
		  : translation_(affine.translation()), rotation_(affine.rotation()) {}
	  Rigid3(const Vector& translation, const Quaternion& rotation)
		  : translation_(translation), rotation_(rotation) {}
	  Rigid3(const Vector& translation, const AngleAxis& rotation)
		  : translation_(translation), rotation_(rotation) {}

	  static Rigid3 Rotation(const AngleAxis& angle_axis) {
		return Rigid3(Vector::Zero(), Quaternion(angle_axis));
	  }

	  static Rigid3 Rotation(const Quaternion& rotation) {
		return Rigid3(Vector::Zero(), rotation);
	  }

	  static Rigid3 Translation(const Vector& vector) {
		return Rigid3(vector, Quaternion::Identity());
	  }

	  static Rigid3<FloatType> Identity() {
		return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
	  }

	  template <typename OtherType>
	  Rigid3<OtherType> cast() const {
		return Rigid3<OtherType>(translation_.template cast<OtherType>(),
								 rotation_.template cast<OtherType>());
	  }

	  const Vector& translation() const { return translation_; }
	  const Quaternion& rotation() const { return rotation_; }

	  Rigid3 inverse() const {
		const Quaternion rotation = rotation_.conjugate();
		const Vector translation = -(rotation * translation_);
		return Rigid3(translation, rotation);
	  }

	  std::string DebugString() const {
		 std::string out;
		out.append("{ t: [");
		out.append(std::to_string(translation().x()));
		out.append(", ");
		out.append(std::to_string(translation().y()));
		out.append(", ");
		out.append(std::to_string(translation().z()));
		out.append("], q: [");
		out.append(std::to_string(rotation().w()));
		out.append(", ");
		out.append(std::to_string(rotation().x()));
		out.append(", ");
		out.append(std::to_string(rotation().y()));
		out.append(", ");
		out.append(std::to_string(rotation().z()));
		out.append("] }");
		return out;
	  }

	 private:
	  Vector translation_;
	  Quaternion rotation_;
	};

	template <typename FloatType>
	Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
								const Rigid3<FloatType>& rhs) {
	  return Rigid3<FloatType>(
		  lhs.rotation() * rhs.translation() + lhs.translation(),
		  lhs.rotation() * rhs.rotation());
	}

	template <typename FloatType>
	typename Rigid3<FloatType>::Vector operator*(
		const Rigid3<FloatType>& rigid,
		const typename Rigid3<FloatType>::Vector& point) {
	  return rigid.rotation() * point + rigid.translation();
	}

	// This is needed for gmock.
	template <typename T>
	std::ostream& operator<<(std::ostream& os,
							 const Rigid3<T>& rigid) {
	  os << rigid.DebugString();
	  return os;
	}

typedef  Rigid3<double> Rigid3d;
typedef  Rigid3<float> Rigid3f;

  class OrientationTrans{
    public:
    static void  quattrans(double* q, double* qt);
    static void  euler2matrix(double* euler, double* matrix);
    static void  matrix2euler(double* matrix,double* euler);
    static void  matrix2quat(double* matrix, double* q);
    static void  euler2quat(double* euler,double* q);
    static void  quat2matrix(double* q, double* matrix);
    static void  quat2euler(double* q, double* euler);
    static void  quatrotvec(double* q, double* n,double* rn);
    static void  quatprod(double* q, double* p, double *qp);
    static void  quatslerp(double*q, double* p, double t, double* qt);
    };
}

