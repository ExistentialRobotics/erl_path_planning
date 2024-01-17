#include "erl_search_planning/reeds_shepp_path.hpp"
#include "erl_common/angle_utils.hpp"
#include "erl_common/polar_coords.hpp"

namespace erl::search_planning {

    const double kZero = -10 * std::numeric_limits<double>::epsilon();

    // 8.1
    inline static bool
    LpSpLp(double x, double y, double phi, double &t, double &u, double &v) {  // left forward, straight forward, left forward
        common::CartesianToPolar(x - std::sin(phi), y - 1. + std::cos(phi), u, t);
        // path is not optimal if t or v is outside [0, pi]
        if (t >= kZero) {
            v = common::WrapAnglePi(phi - t);
            if (v >= kZero) { return true; }
        }
        return false;
    }

    // 8.2
    inline static bool
    LpSpRp(double x, double y, double phi, double &t, double &u, double &v) {  // left forward, straight forward, right forward
        double xi = x + std::sin(phi);
        double eta = y - 1. - std::cos(phi);
        double u1 = xi * xi + eta * eta;
        if (u1 >= 4.0) {
            u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            t = common::WrapAnglePi(std::atan2(eta, xi) + theta);
            v = common::WrapAnglePi(t - phi);
            return t >= kZero && v >= kZero;
        }
        return false;
    }

    void
    CSC(double x,
        double y,
        double phi,
        const ReedsSheppPath::ReedsSheppPathSegmentType *&sol_type,
        double &sol_t,
        double &sol_u,
        double &sol_v,
        double &sol_l) {
        double t, u, v, l;
        if (LpSpLp(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 14: L+S+L+NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[14];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpSpLp(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 14: L-S-L-NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[14];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }
        if (LpSpLp(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 15: R+S+R+NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[15];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpSpLp(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 15: R-S-R-NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[15];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }

        if (LpSpRp(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 12: L+S+R+NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[12];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpSpRp(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 12: L-S-R-NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[12];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }
        if (LpSpRp(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 13: R+S+L+NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[13];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpSpRp(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 13: R-S-L-NN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[13];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }
    }

    // 8.3, 8.4
    inline static bool
    LpRmL(double x, double y, double phi, double &t, double &u, double &v) {  // left forward, right backward, left forward/backward
        double xi = x - std::sin(phi);
        double eta = y - 1. + std::cos(phi);
        double u1 = xi * xi + eta * eta;
        if (u1 <= 16.0) {
            u = -2.0 * std::asin(0.25 * std::sqrt(u1));
            double theta = std::atan2(eta, xi);
            t = common::WrapAnglePi(theta + 0.5 * u + M_PI);
            v = common::WrapAnglePi(phi - t + u);
            return t >= kZero && u <= kZero;
        }
        return false;
    }

    void
    CCC(double x,
        double y,
        double phi,
        const ReedsSheppPath::ReedsSheppPathSegmentType *&sol_type,
        double &sol_t,
        double &sol_u,
        double &sol_v,
        double &sol_l) {
        double t, u, v, l;
        // CCC: (0, 0, 0) -> (x, y, phi)
        if (LpRmL(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 0: L+R-LNN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[0];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpRmL(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 0: L-R+LNN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[0];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }
        if (LpRmL(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 1: R+L-RNN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[1];
            sol_t = t;
            sol_u = u;
            sol_v = v;
            sol_l = l;
        }
        if (LpRmL(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {  // 1: R-L+RNN
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[1];
            sol_t = -t;
            sol_u = -u;
            sol_v = -v;
            sol_l = l;
        }
        // CCC: backward path from (x, y, phi) to (0, 0, 0), then flip the path (order and sign)
        // (x, y, phi) to (0, 0, 0)
        // (0, 0, 0) to (x', y', -phi), x'=-x*cos(phi)-y*sin(phi), y'=x*sin(phi)-y*cos(phi)
        // flip sign, (0, 0, 0) to (-x', y', phi), x'' = -x' = x*cos(phi)+y*sin(phi)
        double xb = x * std::cos(phi) + y * std::sin(phi);
        double yb = x * std::sin(phi) - y * std::cos(phi);
        if (LpRmL(xb, yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[0];
            sol_t = v;
            sol_u = u;
            sol_v = t;
            sol_l = l;
        }
        if (LpRmL(-xb, yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[0];
            sol_t = -v;
            sol_u = -u;
            sol_v = -t;
            sol_l = l;
        }
        if (LpRmL(xb, -yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[1];
            sol_t = v;
            sol_u = u;
            sol_v = t;
            sol_l = l;
        }
        if (LpRmL(-xb, -yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[1];
            sol_t = -v;
            sol_u = -u;
            sol_v = -t;
            sol_l = l;
        }
    }

    // 8.6
    inline void
    TauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega) {
        double delta = common::WrapAnglePi(u - v);
        double a = std::sin(u) - std::sin(delta);
        double b = std::cos(u) - std::cos(delta) - 1.0;
        double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
        double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
        tau = (t2 < 0) ? common::WrapAnglePi(t1 + M_PI) : common::WrapAnglePi(t1);
        omega = common::WrapAnglePi(tau - u + v - phi);
    }

    // 8.7
    inline bool
    LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v) {
        double xi = x + std::sin(phi);
        double eta = y - 1. - std::cos(phi);
        double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
        if (rho <= 1.0) {
            u = std::acos(rho);  // [0, pi/2]
            TauOmega(u, -u, xi, eta, phi, t, v);
            return t >= kZero && v <= -kZero;
        }
        return false;
    }

    // 8.8
    inline bool
    LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v) {
        double xi = x + std::sin(phi);
        double eta = y - 1. - std::cos(phi);
        double rho = (20. - xi * xi - eta * eta) / 16.;
        if (rho >= 0 && rho <= 1) {
            u = -std::acos(rho);  // [-pi/2, 0]
            TauOmega(u, u, xi, eta, phi, t, v);
            return t >= kZero && v >= kZero;
        }
        return false;
    }

    void
    CCCC(
        double x,
        double y,
        double phi,
        const ReedsSheppPath::ReedsSheppPathSegmentType *&sol_type,
        double &sol_t,
        double &sol_u,
        double &sol_v,
        double &sol_w,
        double &sol_l) {
        double t, u, v, l;
        if (LpRupLumRm(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 2: L+R+L-R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[2];
            sol_t = t;
            sol_u = u;
            sol_v = -u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRupLumRm(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 2: L-R-L+R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[2];
            sol_t = -t;
            sol_u = -u;
            sol_v = u;
            sol_w = -v;
            sol_l = l;
        }
        if (LpRupLumRm(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 3: R+L+R-L-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[3];
            sol_t = t;
            sol_u = u;
            sol_v = -u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRupLumRm(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 3: R-L-R+L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[3];
            sol_t = -t;
            sol_u = -u;
            sol_v = u;
            sol_w = -v;
            sol_l = l;
        }

        if (LpRumLumRp(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 2: L+R-L-R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[2];
            sol_t = t;
            sol_u = u;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRumLumRp(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 2: L-R+L+R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[2];
            sol_t = -t;
            sol_u = -u;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }
        if (LpRumLumRp(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 3: R+L-R-L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[3];
            sol_t = t;
            sol_u = u;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRumLumRp(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {  // 3: R-L+R+L-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[3];
            sol_t = -t;
            sol_u = -u;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }
    }

    // 8.9
    inline bool
    LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v) {
        double xi = x - std::sin(phi);
        double eta = y - 1. + std::cos(phi);
        double rho = xi * xi + eta * eta;
        if (rho >= 4.0) {
            double r = std::sqrt(rho - 4.0);
            double theta = std::atan2(eta, xi);
            u = 2.0 - r;
            t = common::WrapAnglePi(theta + std::atan2(r, -2.0));
            v = common::WrapAnglePi(phi - M_PI_2 - t);
            return t >= kZero && u <= -kZero && v <= -kZero;
        }
        return false;
    }

    // 8.10
    inline bool
    LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v) {
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);
        double rho = xi * xi + eta * eta;
        if (rho >= 4.0) {
            rho = std::sqrt(rho);
            t = std::atan2(xi, -eta);
            u = 2.0 - rho;
            v = common::WrapAnglePi(t + M_PI_2 - phi);
            return t >= kZero && u <= -kZero && v <= -kZero;
        }
        return false;
    }

    void
    CCSC(
        double x,
        double y,
        double phi,
        const ReedsSheppPath::ReedsSheppPathSegmentType *&sol_type,
        double &sol_t,
        double &sol_u,
        double &sol_v,
        double &sol_w,
        double &sol_l) {
        double t, u, v, l;
        if (LpRmSmLm(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 4: L+R-S-L-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[4];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRmSmLm(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 4: L-R+S+L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[4];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }
        if (LpRmSmLm(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 5: R+L-S-R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[5];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRmSmLm(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 5: R-L+S+R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[5];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }

        if (LpRmSmRm(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 8: L+R-S-R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[8];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRmSmRm(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 8: L-R+S+R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[8];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }
        if (LpRmSmRm(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 9: R+L-S-L-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[9];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = v;
            sol_l = l;
        }
        if (LpRmSmRm(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 9: R-L+S+L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[9];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = -v;
            sol_l = l;
        }

        // CCSC: backward path from (x, y, phi) to (0, 0, 0), then flip the path (order and sign)
        // (x, y, phi) to (0, 0, 0)
        // (0, 0, 0) to (x', y', -phi), x'=-x*cos(phi)-y*sin(phi), y'=x*sin(phi)-y*cos(phi)
        // flip sign, (0, 0, 0) to (-x', y', phi), x'' = -x' = x*cos(phi)+y*sin(phi)
        double xb = x * std::cos(phi) + y * std::sin(phi);
        double yb = x * std::sin(phi) - y * std::cos(phi);
        if (LpRmSmLm(xb, yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 6: L-S-R-L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[6];
            sol_t = v;
            sol_u = u;
            sol_v = -M_PI_2;
            sol_w = t;
            sol_l = l;
        }
        if (LpRmSmLm(-xb, yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 6: L+S+R+L-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[6];
            sol_t = -v;
            sol_u = -u;
            sol_v = M_PI_2;
            sol_w = -t;
            sol_l = l;
        }
        if (LpRmSmLm(xb, -yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 7: R-S-L-R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[7];
            sol_t = v;
            sol_u = u;
            sol_v = -M_PI_2;
            sol_w = t;
            sol_l = l;
        }
        if (LpRmSmLm(-xb, -yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 7: R+S+L+R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[7];
            sol_t = -v;
            sol_u = -u;
            sol_v = M_PI_2;
            sol_w = -t;
            sol_l = l;
        }

        if (LpRmSmRm(xb, yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 10: R-S-R-L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[10];
            sol_t = v;
            sol_u = u;
            sol_v = -M_PI_2;
            sol_w = t;
            sol_l = l;
        }
        if (LpRmSmRm(-xb, yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 10: R-S-R-L+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[10];
            sol_t = -v;
            sol_u = -u;
            sol_v = M_PI_2;
            sol_w = -t;
            sol_l = l;
        }
        if (LpRmSmRm(xb, -yb, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 11: L-S-L-R+N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[11];
            sol_t = v;
            sol_u = u;
            sol_v = -M_PI_2;
            sol_w = t;
            sol_l = l;
        }
        if (LpRmSmRm(-xb, -yb, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI_2)) {  // 11: L+S+L+R-N
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[11];
            sol_t = -v;
            sol_u = -u;
            sol_v = M_PI_2;
            sol_w = -t;
            sol_l = l;
        }
    }

    // 8.11
    inline bool
    LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v) {
        double xi = x + std::sin(phi);
        double eta = y - 1. - std::cos(phi);
        double rho = xi * xi + eta * eta;
        if (rho >= 4.0) {
            u = 4.0 - std::sqrt(rho - 4.0);
            if (u <= -kZero) {
                t = common::WrapAnglePi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
                v = common::WrapAnglePi(t - phi);
                return t >= kZero && v >= kZero;
            }
        }
        return false;
    }

    void
    CCSCC(
        double x,
        double y,
        double phi,
        const ReedsSheppPath::ReedsSheppPathSegmentType *&sol_type,
        double &sol_t,
        double &sol_u,
        double &sol_v,
        double &sol_w,
        double &sol_z,
        double &sol_l) {
        double t, u, v, l;
        if (LpRmSLmRp(x, y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI)) {  // 16: L+R-S-L-R+
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[16];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = -M_PI_2;
            sol_z = v;
            sol_l = l;
        }
        if (LpRmSLmRp(-x, y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI)) {  // 16: L-R+S+L+R-
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[16];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = M_PI_2;
            sol_z = -v;
            sol_l = l;
        }
        if (LpRmSLmRp(x, -y, -phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI)) {  // 17: R+L-S-R-L+
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[17];
            sol_t = t;
            sol_u = -M_PI_2;
            sol_v = u;
            sol_w = -M_PI_2;
            sol_z = v;
            sol_l = l;
        }
        if (LpRmSLmRp(-x, -y, phi, t, u, v) && sol_l > (l = std::fabs(t) + std::fabs(u) + std::fabs(v) + M_PI)) {  // 17: R-L+S+R+L-
            sol_type = ReedsSheppPath::sk_ReedsSheppPathType_[17];
            sol_t = -t;
            sol_u = M_PI_2;
            sol_v = -u;
            sol_w = M_PI_2;
            sol_z = -v;
            sol_l = l;
        }
    }

    std::shared_ptr<ReedsSheppPath>
    ReedsSheppPath::Create(double x0, double y0, double phi0, double x1, double y1, double phi1, double turning_radius) {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double c = std::cos(phi0);
        double s = std::sin(phi0);
        double x = c * dx + s * dy;
        double y = -s * dx + c * dy;
        double phi = phi1 - phi0;
        x /= turning_radius;
        y /= turning_radius;

        double sol_t, sol_u, sol_v;
        double sol_w = std::numeric_limits<double>::max(), sol_z = std::numeric_limits<double>::max(), sol_l = std::numeric_limits<double>::max();
        const ReedsSheppPathSegmentType *sol_type = nullptr;
        CSC(x, y, phi, sol_type, sol_t, sol_u, sol_v, sol_l);
        CCC(x, y, phi, sol_type, sol_t, sol_u, sol_v, sol_l);
        CCCC(x, y, phi, sol_type, sol_t, sol_u, sol_v, sol_w, sol_l);
        CCSC(x, y, phi, sol_type, sol_t, sol_u, sol_v, sol_w, sol_l);
        CCSCC(x, y, phi, sol_type, sol_t, sol_u, sol_v, sol_w, sol_z, sol_l);

        return std::shared_ptr<ReedsSheppPath>(
            new ReedsSheppPath(x0, y0, phi0, x1, y1, phi1, sol_type, sol_t, sol_u, sol_v, sol_w, sol_z, sol_l, turning_radius));
    }

    void
    ReedsSheppPath::Interpolate(double t, double &x, double &y, double &phi) const {
        if (t <= 0.0) {
            x = m_start_[0];
            y = m_start_[1];
            phi = m_start_[2];
            return;
        }
        if (t >= 1.0) {
            x = m_goal_[0];
            y = m_goal_[1];
            phi = m_goal_[2];
            return;
        }

        x = 0;
        y = 0;
        phi = m_start_[2];

        double remaining_length = m_total_length_ * t;
        for (int seg_type_index = 0; seg_type_index < 5; ++seg_type_index) {

            auto &seg_type = m_type_[seg_type_index];
            double v;
            const double &kSegLength = m_length_[seg_type_index];
            if (kSegLength >= 0) {
                v = std::min(remaining_length, kSegLength);
                remaining_length -= v;
            } else {
                v = std::max(-remaining_length, kSegLength);
                remaining_length += v;
            }

            switch (seg_type) {
                case kReedsSheppLeft:
                    x += std::sin(phi + v) - std::sin(phi);
                    y -= std::cos(phi + v) - std::cos(phi);
                    phi += v;
                    break;
                case kReedsSheppRight:
                    x -= std::sin(phi - v) - std::sin(phi);
                    y += std::cos(phi - v) - std::cos(phi);
                    phi -= v;
                    break;
                case kReedsSheppStraight:
                    x += v * std::cos(phi);
                    y += v * std::sin(phi);
                    break;
                case kReedsSheppNop:
                    seg_type_index = 5;  // terminate the loop
                    break;
            }
        }

        x = x * m_tuning_radius_ + m_start_[0];
        y = y * m_tuning_radius_ + m_start_[1];
    }

    void
    ReedsSheppPath::InterpolateNPoints(std::size_t n, std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &phis) const {
        xs.reserve(n);
        ys.reserve(n);
        phis.reserve(n);

        xs.push_back(m_start_[0]);
        ys.push_back(m_start_[1]);
        phis.push_back(m_start_[2]);

        double x_base = 0;
        double y_base = 0;
        double phi_base = m_start_[2];
        int seg_index = 0;

        auto step = [&x_base, &y_base, &phi_base, &seg_index, this](double v, double &x_out, double &y_out, double &phi_out) {
            switch (m_type_[seg_index]) {
                case kReedsSheppLeft:
                    x_out = x_base + std::sin(phi_base + v) - std::sin(phi_base);
                    y_out = y_base - std::cos(phi_base + v) + std::cos(phi_base);
                    phi_out = phi_base + v;
                    break;
                case kReedsSheppRight:
                    x_out = x_base - std::sin(phi_base - v) + std::sin(phi_base);
                    y_out = y_base + std::cos(phi_base - v) - std::cos(phi_base);
                    phi_out = phi_base - v;
                    break;
                case kReedsSheppStraight:
                    x_out = x_base + v * std::cos(phi_base);
                    y_out = y_base + v * std::sin(phi_base);
                    phi_out = phi_base;
                    break;
                case kReedsSheppNop:
                    break;
            }
            phi_out = common::WrapAnglePi(phi_out);  // phi should be in [-pi, pi)
        };

        double s = 0;
        double ds = m_total_length_ / double(n - 1);
        double x, y, phi;
        while (m_type_[seg_index] != kReedsSheppNop) {
            s += ds;
            double seg_length = std::fabs(m_length_[seg_index]);
            if (s >= seg_length) {
                step(m_length_[seg_index], x_base, y_base, phi_base);  // move to the end of the segment, i.e. the start of the next segment
                s -= seg_length;
                ++seg_index;
            }

            if (m_type_[seg_index] == kReedsSheppNop) {
                x = x_base;
                y = y_base;
                phi = phi_base;
                ERL_DEBUG_ASSERT(
                    std::fabs(x_base * m_tuning_radius_ + m_start_[0] - m_goal_[0]) < 1.e-6,
                    "x_base = %f, m_goal_[0] = %f before interpolation ends.",
                    x_base,
                    m_goal_[0]);
                ERL_DEBUG_ASSERT(
                    std::fabs(y_base * m_tuning_radius_ + m_start_[1] - m_goal_[1]) < 1.e-6,
                    "y_base = %f, m_goal_[1] = %f before interpolation ends.",
                    y_base,
                    m_goal_[1]);
                ERL_DEBUG_ASSERT(std::fabs(phi_base - m_goal_[2]) < 1.e-6, "phi_base = %f, m_goal_[2] = %f before interpolation ends.", phi_base, m_goal_[2]);
            } else if (m_length_[seg_index] >= 0) {
                step(s, x, y, phi);
            } else {
                step(-s, x, y, phi);
            }
            xs.push_back(x * m_tuning_radius_ + m_start_[0]);
            ys.push_back(y * m_tuning_radius_ + m_start_[1]);
            phis.push_back(phi);
        }
    }
}  // namespace erl::search_planning
