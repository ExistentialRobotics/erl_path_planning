#include "erl_search_planning/reeds_shepp_path.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_common/opencv.hpp"
#include <memory>

struct AppData {
    std::string window_name = "ReedsSheppPath";
    cv::Mat canvas = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
    double x0 = 0, y0 = 0, phi0 = 0, x1 = 0, y1 = 0, phi1 = 0;
    bool start_set = false, goal_set = false;
    bool l_button_down = false;
    bool m_button_down = false;
    std::shared_ptr<erl::search_planning::ReedsSheppPath> reeds_shepp_path = nullptr;
    std::size_t n_points = 100;
    int dir_vec_len = 50;
    double turning_radius = 30.0;
};

static void
OnMouse(int event, int x, int y, int flags, void *userdata) {
    (void) flags;
    auto *app_data = static_cast<AppData *>(userdata);
    int y_cv = y;
    y = app_data->canvas.rows - y;
    if (event == cv::EVENT_LBUTTONDOWN) {
        app_data->x0 = x;
        app_data->y0 = y;
        app_data->start_set = false;
        app_data->l_button_down = true;
        app_data->reeds_shepp_path = nullptr;
    } else if (event == cv::EVENT_MBUTTONDOWN) {
        app_data->x1 = x;
        app_data->y1 = y;
        app_data->goal_set = false;
        app_data->m_button_down = true;
        app_data->reeds_shepp_path = nullptr;
    } else if (event == cv::EVENT_LBUTTONUP) {
        app_data->phi0 = std::atan2(double(y - app_data->y0), double(x - app_data->x0));
        app_data->start_set = true;
        app_data->l_button_down = false;
        ERL_INFO("set start: (%f, %f, %f)", app_data->x0, app_data->y0, app_data->phi0);
    } else if (event == cv::EVENT_MBUTTONUP) {
        app_data->phi1 = std::atan2(double(y - app_data->y1), double(x - app_data->x1));
        app_data->goal_set = true;
        app_data->m_button_down = false;
        ERL_INFO("set goal: (%f, %f, %f)", app_data->x1, app_data->y1, app_data->phi1);
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (!app_data->l_button_down && !app_data->m_button_down) { return; }
    }

    if (app_data->reeds_shepp_path != nullptr) { return; }  // already computed and drawn

    cv::Mat tmp;
    if (app_data->start_set) {
        double dx = double(app_data->dir_vec_len) * std::cos(app_data->phi0);
        double dy = double(app_data->dir_vec_len) * std::sin(app_data->phi0);
        cv::Point2i p1(int(app_data->x0), app_data->canvas.rows - int(app_data->y0));
        cv::Point2i p2(int(std::round(app_data->x0 + dx)), app_data->canvas.rows - int(std::round(app_data->y0 + dy)));
        cv::arrowedLine(app_data->canvas, p1, p2, cv::Scalar(0, 0, 0), 2);
        cv::circle(app_data->canvas, p1, 3, cv::Scalar(0, 255, 0), -1);
    } else if (app_data->l_button_down) {
        tmp = app_data->canvas.clone();
        cv::Point2i p1(int(app_data->x0), app_data->canvas.rows - int(app_data->y0));
        cv::Point2i p2(x, y_cv);
        cv::arrowedLine(tmp, p1, p2, cv::Scalar(0, 0, 0), 2);
        cv::circle(tmp, p1, 3, cv::Scalar(0, 255, 0), -1);
    }

    if (app_data->goal_set) {
        double dx = double(app_data->dir_vec_len) * std::cos(app_data->phi1);
        double dy = double(app_data->dir_vec_len) * std::sin(app_data->phi1);
        cv::Point2i p1(int(app_data->x1), app_data->canvas.rows - int(app_data->y1));
        cv::Point2i p2(int(std::round(app_data->x1 + dx)), app_data->canvas.rows - int(std::round(app_data->y1 + dy)));
        cv::arrowedLine(app_data->canvas, p1, p2, cv::Scalar(0, 0, 0), 2);
        cv::circle(app_data->canvas, p1, 3, cv::Scalar(0, 255, 0), -1);
    } else if (app_data->m_button_down) {
        tmp = app_data->canvas.clone();
        cv::Point2i p1(int(app_data->x1), app_data->canvas.rows - int(app_data->y1));
        cv::Point2i p2(x, y_cv);
        cv::arrowedLine(tmp, p1, p2, cv::Scalar(0, 0, 0), 2);
        cv::circle(tmp, p1, 3, cv::Scalar(0, 255, 0), -1);
    }

    if (!app_data->start_set || !app_data->goal_set) {
        if (tmp.rows > 0) {
            cv::imshow(app_data->window_name, tmp);
        } else {
            cv::imshow(app_data->window_name, app_data->canvas);
        }
        return;
    }

    app_data->reeds_shepp_path = erl::search_planning::ReedsSheppPath::Create(
        app_data->x0,
        app_data->y0,
        app_data->phi0,
        app_data->x1,
        app_data->y1,
        app_data->phi1,
        app_data->turning_radius);
    ERL_INFO(
        "dubins path: %s, length: %f (t=%f, u=%f, v=%f, w=%f, z=%f)",
        app_data->reeds_shepp_path->GetReedsSheppPathType().c_str(),
        app_data->reeds_shepp_path->GetLength(),
        app_data->reeds_shepp_path->GetSegmentLength(0),
        app_data->reeds_shepp_path->GetSegmentLength(1),
        app_data->reeds_shepp_path->GetSegmentLength(2),
        app_data->reeds_shepp_path->GetSegmentLength(3),
        app_data->reeds_shepp_path->GetSegmentLength(4));
    std::vector<double> xs, ys, phis;
    app_data->reeds_shepp_path->InterpolateNPoints(app_data->n_points, xs, ys, phis);
    std::vector<cv::Point> points;
    points.reserve(app_data->n_points);
    for (std::size_t i = 0; i < app_data->n_points; ++i) {
        double dx = double(app_data->dir_vec_len) * std::cos(phis[i]);
        double dy = double(app_data->dir_vec_len) * std::sin(phis[i]);
        points.emplace_back(xs[i], app_data->canvas.rows - ys[i]);
        auto x2 = int(std::round(xs[i] + dx));
        auto y2 = app_data->canvas.rows - int(std::round(ys[i] + dy));
        cv::arrowedLine(app_data->canvas, points.back(), cv::Point2i(x2, y2), cv::Scalar(255, 0, 0), 1);
    }
    cv::polylines(app_data->canvas, points, false, cv::Scalar(0, 0, 255), 2);
    cv::circle(app_data->canvas, points.front(), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(app_data->canvas, points.back(), 3, cv::Scalar(0, 255, 0), -1);
    cv::imshow(app_data->window_name, app_data->canvas);
}

TEST(HybridAStar, DubinsPath) {
    AppData app_data;
    cv::imshow(app_data.window_name, app_data.canvas);
    cv::setMouseCallback(app_data.window_name, OnMouse, &app_data);
    while (true) {
        int key = cv::waitKey(10);
        if (key == 27 || key == 'q') { break; }
        if (key == 'c') {
            app_data.canvas.setTo(cv::Scalar(255, 255, 255));
            app_data.start_set = false;
            app_data.goal_set = false;
            app_data.l_button_down = false;
            app_data.m_button_down = false;
            cv::imshow(app_data.window_name, app_data.canvas);
        }
    }
    cv::destroyAllWindows();
}
