#pragma once

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>


namespace ORB_SLAM2
{

class Geometry
{
public:
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> YPR2R(
            const Eigen::MatrixBase<Derived> &ypr) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> R2YPR(
            const Eigen::MatrixBase<Derived> &R) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(
            const Eigen::MatrixBase<Derived> &v) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> LogSO3(
            const Eigen::MatrixBase<Derived> &R) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ExpSO3(
            const Eigen::MatrixBase<Derived> &v) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
            const Eigen::QuaternionBase<Derived> &Q) noexcept;

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
            const Eigen::QuaternionBase<Derived> &Q) noexcept;

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> DeltaQuat(
            const Eigen::MatrixBase<Derived> &theta) noexcept;

    template <typename Derived>
    static void InterpolateOrientation(
            const double &ts0, const Eigen::MatrixBase<Derived> &R1,
            const double &ts1, const Eigen::MatrixBase<Derived> &R2,
            const double &ts, Eigen::MatrixBase<Derived> &R) noexcept;

    template <typename Derived>
    static void InterpolatePosition(
            const double &ts0, const Eigen::MatrixBase<Derived> &P1,
            const double &ts1, const Eigen::MatrixBase<Derived> &P2,
            const double &ts, Eigen::MatrixBase<Derived> &P) noexcept;

};

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Geometry::YPR2R(
        const Eigen::MatrixBase<Derived> &ypr) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
        0., 1., 0.,
        -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> Geometry::R2YPR(
        const Eigen::MatrixBase<Derived> &R) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Matrix<Scalar_t, 3, 1> n = R.col(0);
    Eigen::Matrix<Scalar_t, 3, 1> o = R.col(1);
    Eigen::Matrix<Scalar_t, 3, 1> a = R.col(2);

    Eigen::Matrix<Scalar_t, 3, 1> ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Geometry::Skew(
        const Eigen::MatrixBase<Derived> &v) noexcept
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -v(2), v(1),
        v(2), typename Derived::Scalar(0), -v(0),
        -v(1), v(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> Geometry::LogSO3(
        const Eigen::MatrixBase<Derived> &R) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::AngleAxis<Scalar_t> a(R);
    Eigen::Matrix<Scalar_t, 3, 1> v = a.angle() * a.axis();

    return v;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Geometry::ExpSO3(
        const Eigen::MatrixBase<Derived> &v) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::AngleAxis<Scalar_t> a(v.norm(), v.normalized());

    return a.toRotationMatrix();
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> Geometry::Qleft(
        const Eigen::QuaternionBase<Derived> &Q) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    auto I3 = Eigen::Matrix<Scalar_t, 3, 3>::Identity();

    Eigen::Matrix<Scalar_t, 4, 4> M;
    M(0, 0) = Q.w();
    M.template block<1, 3>(0, 1) = -Q.vec().transpose();
    M.template block<3, 1>(1, 0) = Q.vec();
    M.template block<3, 3>(1, 1) = Q.w() * I3 + Skew(Q.vec());
    return M;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> Geometry::Qright(
        const Eigen::QuaternionBase<Derived> &Q) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    auto I3 = Eigen::Matrix<Scalar_t, 3, 3>::Identity();

    Eigen::Matrix<Scalar_t, 4, 4> M;
    M(0, 0) = Q.w();
    M.template block<1, 3>(0, 1) = -Q.vec().transpose();
    M.template block<3, 1>(1, 0) = Q.vec();
    M.template block<3, 3>(1, 1) = Q.w() * I3 - Skew(Q.vec());
    return M;
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> Geometry::DeltaQuat(
        const Eigen::MatrixBase<Derived> &theta) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template <typename Derived>
void Geometry::InterpolateOrientation(
        const double &ts0, const Eigen::MatrixBase<Derived> &R0,
        const double &ts1, const Eigen::MatrixBase<Derived> &R1,
        const double &ts, Eigen::MatrixBase<Derived> &R) noexcept
{
    typedef typename Derived::Scalar Scalar_t;

    double w0 = (ts - ts0) / (ts1 - ts0);
    double w1 = 1 - w0;
    Eigen::Quaternion<Scalar_t> Q0(R0);
    Eigen::Quaternion<Scalar_t> Q1(R1);
    Eigen::Quaternion<Scalar_t> Q = Q0.slerp(w1, Q1);
    R = Q.toRotationMatrix();
}

template <typename Derived>
void Geometry::InterpolatePosition(
        const double &ts0, const Eigen::MatrixBase<Derived> &P0,
        const double &ts1, const Eigen::MatrixBase<Derived> &P1,
        const double &ts, Eigen::MatrixBase<Derived> &P) noexcept
{
    double w0 = (ts - ts0) / (ts1 - ts0);
    double w1 = 1 - w0;
    P = w0 * P0 + w1 * P1;
}

} // namespace ORB_SLAM2
