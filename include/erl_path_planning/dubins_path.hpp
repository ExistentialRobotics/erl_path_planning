#pragma once
#include "erl_common/logging.hpp"
#include "erl_env/environment_state.hpp"

namespace erl::path_planning {

    class DubinsPath {

    public:
        enum DubinsPathSegmentType { kDubinsLeft = 0, kDubinsStraight = 1, kDubinsRight = 2 };

        inline static const DubinsPathSegmentType sk_DubinsPathType_[6][3] = {
            {kDubinsLeft, kDubinsStraight, kDubinsLeft},    // LSL
            {kDubinsRight, kDubinsStraight, kDubinsRight},  // RSR
            {kDubinsRight, kDubinsStraight, kDubinsLeft},   // RSL
            {kDubinsLeft, kDubinsStraight, kDubinsRight},   // LSR
            {kDubinsRight, kDubinsLeft, kDubinsRight},      // RLR
            {kDubinsLeft, kDubinsRight, kDubinsLeft}};      // LRL

        static std::shared_ptr<DubinsPath>
        Create(
            double x0,
            double y0,
            double phi0,
            double x1,
            double y1,
            double phi1,
            double turning_radius = 1.0);

        void
        Interpolate(double t, double &x, double &y, double &phi) const;

        void
        InterpolateNPoints(
            std::size_t n,
            std::vector<double> &xs,
            std::vector<double> &ys,
            std::vector<double> &phis) const;

        [[nodiscard]] double
        GetLength() const {
            return m_total_length_ * m_tuning_radius_;
        }

        [[nodiscard]] double
        GetSegmentLength(int i) const {
            ERL_ASSERTM(i >= 0 && i < 3, "i = %d should be in [0, 3).", i);
            return m_length_[i] * m_tuning_radius_;
        }

        [[nodiscard]] double
        GetTurningRadius() const {
            return m_tuning_radius_;
        }

        [[nodiscard]] std::string
        GetDubinsPathType() const {
            static auto table = "LSR";
            char type[3] = {0, 0, 0};
            for (int i = 0; i < 3; ++i) { type[i] = table[m_type_[i]]; }
            return type;
        }

    private:
        explicit DubinsPath(
            const double x0,
            const double y0,
            const double phi0,
            const double x1,
            const double y1,
            const double phi1,
            const DubinsPathSegmentType *type = nullptr,
            double t = std::numeric_limits<double>::max(),
            double p = std::numeric_limits<double>::max(),
            double q = std::numeric_limits<double>::max(),
            const double total_length = std::numeric_limits<double>::max(),
            const double tuning_radius = 1.0)
            : m_type_(type),
              m_total_length_(total_length),
              m_tuning_radius_(tuning_radius) {
            m_start_[0] = x0;
            m_start_[1] = y0;
            m_start_[2] = phi0;
            m_goal_[0] = x1;
            m_goal_[1] = y1;
            m_goal_[2] = phi1;
            m_length_[0] = t;
            m_length_[1] = p;
            m_length_[2] = q;
            ERL_ASSERTM(t >= 0., "t = {} should be non-negative.", t);
            ERL_ASSERTM(p >= 0., "p = {} should be non-negative.", p);
            ERL_ASSERTM(q >= 0., "q = {} should be non-negative.", q);
        }

        double m_start_[3] = {0, 0, 0};                               // start state
        double m_goal_[3] = {0, 0, 0};                                // end state
        const DubinsPathSegmentType *m_type_;                         // path segment type
        double m_length_[3] = {0, 0, 0};                              // path segment lengths
        double m_total_length_ = std::numeric_limits<double>::max();  // total length
        double m_tuning_radius_ = 1.0;                                // turning radius
    };
}  // namespace erl::path_planning
