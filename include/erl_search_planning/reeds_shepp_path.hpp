#pragma once

#include "erl_common/logging.hpp"
#include "erl_common/template_helper.hpp"

namespace erl::search_planning {

    class ReedsSheppPath {

    public:
        enum ReedsSheppPathSegmentType { kReedsSheppNop = 0, kReedsSheppLeft = 1, kReedsSheppStraight = 2, kReedsSheppRight = 3 };

        inline static const ReedsSheppPathSegmentType sk_ReedsSheppPathType_[18][5] = {
            {kReedsSheppLeft, kReedsSheppRight, kReedsSheppLeft, kReedsSheppNop, kReedsSheppNop},         // 0: LRLNN
            {kReedsSheppRight, kReedsSheppLeft, kReedsSheppRight, kReedsSheppNop, kReedsSheppNop},        // 1: RLRNN
            {kReedsSheppLeft, kReedsSheppRight, kReedsSheppLeft, kReedsSheppRight, kReedsSheppNop},       // 2: LRLRN
            {kReedsSheppRight, kReedsSheppLeft, kReedsSheppRight, kReedsSheppLeft, kReedsSheppNop},       // 3: RLRLN
            {kReedsSheppLeft, kReedsSheppRight, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppNop},    // 4: LRSLN
            {kReedsSheppRight, kReedsSheppLeft, kReedsSheppStraight, kReedsSheppRight, kReedsSheppNop},   // 5: RLSRN
            {kReedsSheppLeft, kReedsSheppStraight, kReedsSheppRight, kReedsSheppLeft, kReedsSheppNop},    // 6: LSRLN
            {kReedsSheppRight, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppRight, kReedsSheppNop},   // 7: RSLRN
            {kReedsSheppLeft, kReedsSheppRight, kReedsSheppStraight, kReedsSheppRight, kReedsSheppNop},   // 8: LRSRN
            {kReedsSheppRight, kReedsSheppLeft, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppNop},    // 9: RLSLN
            {kReedsSheppRight, kReedsSheppStraight, kReedsSheppRight, kReedsSheppLeft, kReedsSheppNop},   // 10: RSRLN
            {kReedsSheppLeft, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppRight, kReedsSheppNop},    // 11: LSLRN
            {kReedsSheppLeft, kReedsSheppStraight, kReedsSheppRight, kReedsSheppNop, kReedsSheppNop},     // 12: LSRNN
            {kReedsSheppRight, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppNop, kReedsSheppNop},     // 13: RSLNN
            {kReedsSheppLeft, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppNop, kReedsSheppNop},      // 14: LSLNN
            {kReedsSheppRight, kReedsSheppStraight, kReedsSheppRight, kReedsSheppNop, kReedsSheppNop},    // 15: RSRNN
            {kReedsSheppLeft, kReedsSheppRight, kReedsSheppStraight, kReedsSheppLeft, kReedsSheppRight},  // 16: LRSLR
            {kReedsSheppRight, kReedsSheppLeft, kReedsSheppStraight, kReedsSheppRight, kReedsSheppLeft}   // 17: RLSRL
        };

        static std::shared_ptr<ReedsSheppPath>
        Create(double x0, double y0, double phi0, double x1, double y1, double phi1, double turning_radius = 1.0);

        void
        Interpolate(double t, double &x, double &y, double &phi) const;

        void
        InterpolateNPoints(std::size_t n, std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &phis) const;

        [[nodiscard]] double
        GetLength() const {
            return m_total_length_ * m_tuning_radius_;
        }

        [[nodiscard]] double
        GetSegmentLength(int i) const {
            ERL_ASSERTM(i >= 0 && i < 5, "i = %d should be in [0, 5).", i);
            return m_length_[i] * m_tuning_radius_;
        }

        [[nodiscard]] double
        GetTurningRadius() const {
            return m_tuning_radius_;
        }

        [[nodiscard]] std::string
        GetReedsSheppPathType() const {
            static auto *table = "NLSR";
            char type[5] = {0, 0, 0, 0, 0};
            for (int i = 0; i < 5; ++i) { type[i] = table[m_type_[i]]; }
            return type;
        }

    private:
        explicit ReedsSheppPath(
            const double x0,
            const double y0,
            const double phi0,
            const double x1,
            const double y1,
            const double phi1,
            const ReedsSheppPathSegmentType *type = nullptr,
            const double t = std::numeric_limits<double>::max(),
            const double u = std::numeric_limits<double>::max(),
            const double v = std::numeric_limits<double>::max(),
            const double w = std::numeric_limits<double>::max(),
            const double z = std::numeric_limits<double>::max(),
            const double total_length = std::numeric_limits<double>::max(),
            const double turning_radius = 1.0)
            : m_type_(type),
              m_total_length_(total_length),
              m_tuning_radius_(turning_radius) {
            m_start_[0] = x0;
            m_start_[1] = y0;
            m_start_[2] = phi0;
            m_goal_[0] = x1;
            m_goal_[1] = y1;
            m_goal_[2] = phi1;
            m_length_[0] = t;
            m_length_[1] = u;
            m_length_[2] = v;
            m_length_[3] = w;
            m_length_[4] = z;
        }

        double m_start_[3] = {0, 0, 0};                               // start state
        double m_goal_[3] = {0, 0, 0};                                // goal state
        const ReedsSheppPathSegmentType *m_type_;                     // path segment type
        double m_length_[5] = {0, 0, 0, 0, 0};                        // path segment lengths
        double m_total_length_ = std::numeric_limits<double>::max();  // total length
        double m_tuning_radius_ = 1.0;                                // turning radius
    };

}  // namespace erl::search_planning
