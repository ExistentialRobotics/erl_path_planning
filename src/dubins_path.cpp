#include "erl_search_planning/dubins_path.hpp"

#include "erl_common/angle_utils.hpp"

namespace erl::search_planning {

    std::shared_ptr<DubinsPath>
    DubinsPath::Create(double x0, double y0, double phi0, double x1, double y1, double phi1, double turning_radius) {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double d = std::sqrt(dx * dx + dy * dy) / turning_radius;
        ERL_ASSERTM(-M_PI <= phi0 && phi0 <= M_PI, "yaw of start state should be in [-pi, pi].");
        ERL_ASSERTM(-M_PI <= phi1 && phi1 <= M_PI, "yaw of goal state should be in [-pi, pi].");
        double phi = std::atan2(dy, dx);
        double alpha = common::WrapAngleTwoPi(phi0 - phi);
        double beta = common::WrapAngleTwoPi(phi1 - phi);

        constexpr double zero = -1.e-7;

        if (constexpr double epsilon = 1.e-6; d < epsilon && std::fabs(alpha - beta) < epsilon) {
            return std::shared_ptr<DubinsPath>(new DubinsPath(x0, y0, phi0, x1, y1, phi1, DubinsPath::sk_DubinsPathType_[0], 0, d, 0, d, turning_radius));
        }

        double ca = std::cos(alpha);
        double sa = std::sin(alpha);
        double cb = std::cos(beta);
        double sb = std::sin(beta);

        double theta, t, p, q, l;
        double sol_t, sol_p, sol_q, sol_l;
        const DubinsPathSegmentType *sol_type;

        // LSL
        dx = d + sa - sb;
        dy = cb - ca;
        theta = std::atan2(dy, dx);
        sol_type = DubinsPath::sk_DubinsPathType_[0];
        sol_t = t = common::WrapAngleTwoPi(theta - alpha);
        sol_p = p = std::sqrt(dx * dx + dy * dy);
        sol_q = q = common::WrapAngleTwoPi(beta - theta);
        sol_l = t + p + q;
        // RSR
        dx = d - sa + sb;
        dy = -dy;  // dy = ca - cb;
        theta = std::atan2(dy, dx);
        t = common::WrapAngleTwoPi(alpha - theta);
        p = std::sqrt(dx * dx + dy * dy);
        q = common::WrapAngleTwoPi(theta - beta);
        l = t + p + q;
        if (l < sol_l) {
            sol_type = DubinsPath::sk_DubinsPathType_[1];
            sol_t = t;
            sol_p = p;
            sol_q = q;
            sol_l = l;
        }
        // RSL
        dx = d - sa - sb;
        dy = ca + cb;
        p = dx * dx + dy * dy - 4;
        if (p >= zero) {
            p = std::sqrt(std::max(p, 0.0));
            theta = std::atan2(dy, dx) - std::atan2(2., p);
            t = common::WrapAngleTwoPi(alpha - theta);
            q = common::WrapAngleTwoPi(beta - theta);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[2];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
        }
        // LSR
        dx = d + sa + sb;
        dy = -dy;  // dy = -ca - cb;
        p = dx * dx + dy * dy - 4;
        if (p >= zero) {
            p = std::sqrt(std::max(p, 0.0));
            theta = std::atan2(dy, dx) - std::atan2(-2., p);
            t = common::WrapAngleTwoPi(theta - alpha);
            q = common::WrapAngleTwoPi(theta - beta);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[3];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
        }
        // RLR
        dx = d - sa + sb;
        dy = ca - cb;
        p = 1.0 - 0.125 * (dx * dx + dy * dy);
        if (std::abs(p) <= 1.0) {  // solution exists
            theta = std::atan2(dy, dx);
            // solution 1
            p = std::acos(p);
            t = common::WrapAngleTwoPi(alpha - theta + p * 0.5);
            q = common::WrapAngleTwoPi(-beta + theta + p * 0.5);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[4];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
            // solution 2
            p = common::WrapAngleTwoPi(2 * M_PI - p);
            t = common::WrapAngleTwoPi(alpha - theta + p * 0.5);
            q = common::WrapAngleTwoPi(alpha - beta - t + p);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[4];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
        }
        // LRL
        dx = d + sa - sb;
        dy = -dy;  // dy = -ca + cb;
        p = 1.0 - 0.125 * (dx * dx + dy * dy);
        if (std::abs(p) <= 1.0) {
            theta = std::atan2(dy, dx);
            // solution 1
            p = std::acos(p);
            t = common::WrapAngleTwoPi(-alpha + theta + p * 0.5);
            q = common::WrapAngleTwoPi(beta - theta + p * 0.5);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[5];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
            // solution 2
            p = common::WrapAngleTwoPi(2 * M_PI - p);
            t = common::WrapAngleTwoPi(-alpha + theta + p * 0.5);
            q = common::WrapAngleTwoPi(beta - alpha - t + p);
            l = t + p + q;
            if (l < sol_l) {
                sol_type = DubinsPath::sk_DubinsPathType_[5];
                sol_t = t;
                sol_p = p;
                sol_q = q;
                sol_l = l;
            }
        }

        return std::shared_ptr<DubinsPath>(new DubinsPath(x0, y0, phi0, x1, y1, phi1, sol_type, sol_t, sol_p, sol_q, sol_l, turning_radius));
    }

    void
    DubinsPath::Interpolate(const double t, double &x, double &y, double &phi) const {
        if (t <= 0.0) {
            x = m_start_[0];
            y = m_start_[1];
            phi = m_start_[2];
            return;
        }
        if (t >= m_total_length_) {
            x = m_goal_[0];
            y = m_goal_[1];
            phi = m_goal_[2];
            return;
        }

        x = 0;
        y = 0;
        phi = m_start_[2];

        double remaining_length = t * m_total_length_;
        for (int seg_type_index = 0; seg_type_index < 3; ++seg_type_index) {
            auto &seg_type = m_type_[seg_type_index];
            const double v = std::min(remaining_length, m_length_[seg_type_index]);
            remaining_length -= v;
            switch (seg_type) {
                case kDubinsLeft:
                    x += std::sin(phi + v) - std::sin(phi);
                    y -= std::cos(phi + v) - std::cos(phi);
                    phi += v;
                    break;
                case kDubinsRight:
                    x -= std::sin(phi - v) - std::sin(phi);
                    y += std::cos(phi - v) - std::cos(phi);
                    phi -= v;
                    break;
                case kDubinsStraight:
                    x += std::cos(phi) * v;
                    y += std::sin(phi) * v;
                    break;
            }
        }

        x = x * m_tuning_radius_ + m_start_[0];
        y = y * m_tuning_radius_ + m_start_[1];
    }

    void
    DubinsPath::InterpolateNPoints(const std::size_t n, std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &phis) const {
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

        auto step = [&x_base, &y_base, &phi_base, &seg_index, this](const double v, double &x_out, double &y_out, double &phi_out) {
            switch (m_type_[seg_index]) {
                case kDubinsLeft:
                    x_out = x_base + std::sin(phi_base + v) - std::sin(phi_base);
                    y_out = y_base - std::cos(phi_base + v) + std::cos(phi_base);
                    phi_out = phi_base + v;
                    break;
                case kDubinsRight:
                    x_out = x_base - std::sin(phi_base - v) + std::sin(phi_base);
                    y_out = y_base + std::cos(phi_base - v) - std::cos(phi_base);
                    phi_out = phi_base - v;
                    break;
                case kDubinsStraight:
                    x_out = x_base + std::cos(phi_base) * v;
                    y_out = y_base + std::sin(phi_base) * v;
                    phi_out = phi_base;
                    break;
            }
            phi_out = common::WrapAnglePi(phi_out);  // phi should be in [-pi, pi)
        };

        double s = 0;
        const double ds = m_total_length_ / static_cast<double>(n - 1);
        double x, y, phi;
        while (seg_index < 3) {
            s += ds;
            if (const double seg_length = std::fabs(m_length_[seg_index]); s > seg_length) {
                step(m_length_[seg_index], x_base, y_base, phi_base);
                s -= seg_length;
                ++seg_index;
            }

            if (seg_index == 3) {
                x = x_base;
                y = y_base;
                phi = phi_base;
                ERL_DEBUG_ASSERT(
                    std::fabs(x_base * m_tuning_radius_ + m_start_[0] - m_goal_[0]) < 1.e-6,
                    "x_base = {}, m_goal_[0] = {} before interpolation ends.",
                    x_base,
                    m_goal_[0]);
                ERL_DEBUG_ASSERT(
                    std::fabs(y_base * m_tuning_radius_ + m_start_[1] - m_goal_[1]) < 1.e-6,
                    "y_base = {}, m_goal_[1] = {} before interpolation ends.",
                    y_base,
                    m_goal_[1]);
                ERL_DEBUG_ASSERT(std::fabs(phi_base - m_goal_[2]) < 1.e-6, "phi_base = {}, m_goal_[2] = {} before interpolation ends.", phi_base, m_goal_[2]);
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
