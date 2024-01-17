import matplotlib.pyplot as plt
import numpy as np
import unittest


def get_circle(center, radius, n=100):
    t = np.linspace(0, 2 * np.pi, n)
    x = center[0] + radius * np.cos(t)
    y = center[1] + radius * np.sin(t)
    return x, y


def rotate_point(point, center, angle):
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    ca = np.cos(angle)
    sa = np.sin(angle)
    x = center[0] + ca * dx - sa * dy
    y = center[1] + sa * dx + ca * dy
    return np.array([x, y])


def get_arc(center, start, angle, n=50):
    t = np.linspace(0, angle, n)
    theta = np.arctan2(start[1] - center[1], start[0] - center[0])
    ct = np.cos(t)
    st = np.sin(t)
    x = center[0] + np.cos(theta) * ct - np.sin(theta) * st
    y = center[1] + np.sin(theta) * ct + np.cos(theta) * st
    return x, y


def mod2pi(theta):
    if 0 > theta > -1.0e-7:
        return 0.0
    theta = theta - 2 * np.pi * np.floor(theta / (2 * np.pi))
    if 2 * np.pi - theta < 5.0e-7:
        return 0.0
    return theta


class VisualizeDubinsPath(unittest.TestCase):
    def test_lrl(self):
        d = 2.0
        alpha = np.deg2rad(90)
        beta = np.deg2rad(270)

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        cb = np.cos(beta)
        sb = np.sin(beta)
        c1 = np.array([-sa, ca])
        c2 = np.array([d - sb, cb])

        dx = d + sa - sb
        dy = cb - ca
        theta = np.arctan2(dy, dx)
        p = 1.0 - 0.125 * (dx * dx + dy * dy)
        print("cos(p) = ", p)
        p = np.arccos(p)
        t = mod2pi(-alpha + theta + p / 2.0)
        q = mod2pi(beta - theta + p / 2.0)
        print("t = ", t)
        print("p = ", p)
        print("q = ", q)
        print("l = ", t + p + q)

        touch_pt1 = rotate_point([0, 0], c1, t)
        arc1 = get_arc(c1, [0, 0], t)
        c3 = 2 * touch_pt1 - c1
        arc2 = get_arc(c3, touch_pt1, -p)
        arc3 = get_arc(c2, [d, 0], -q)

        plt.scatter([c1[0], c2[0], c3[0]], [c1[1], c2[1], c3[1]], color="k")
        plt.plot(*arc1, "k")
        plt.plot(*arc2, "k")
        plt.plot(*arc3, "k")

        p = 2 * np.pi - p
        t = mod2pi(-alpha + theta + p / 2.0)
        q = mod2pi(beta - alpha - t + p)
        print("t = ", t)
        print("p = ", p)
        print("q = ", q)
        print("l = ", t + p + q)

        touch_pt1 = rotate_point([0, 0], c1, t)
        arc1 = get_arc(c1, [0, 0], t)
        c3 = 2 * touch_pt1 - c1
        arc2 = get_arc(c3, touch_pt1, -p)
        arc3 = get_arc(c2, [d, 0], -q)

        plt.plot(*arc1, "g")
        plt.plot(*arc2, "g")
        plt.plot(*arc3, "g")

        plt.plot(*get_circle(c1, 1.0), "b--")
        plt.plot(*get_circle(c2, 1.0), "r--")
        plt.quiver(0, 0, 2 * ca, 2 * sa, color="b")
        plt.quiver(d, 0, 2 * cb, 2 * sb, color="r")
        plt.scatter([0, d], [0, 0], color="r")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

    def test_rlr(self):
        # d = 2.0 * (1 + np.sqrt(2))
        d = 2.0
        alpha = np.deg2rad(90)
        beta = np.deg2rad(270)

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        cb = np.cos(beta)
        sb = np.sin(beta)
        c1 = np.array([sa, -ca])
        c2 = np.array([d + sb, -cb])

        dx = d - sa + sb
        dy = ca - cb
        theta = np.arctan2(dy, dx)
        p = 1.0 - 0.125 * (dx * dx + dy * dy)
        print("cos(p) = ", p)
        p = np.arccos(p)
        t = mod2pi(alpha - theta + p / 2.0)
        q = mod2pi(-beta + theta + p / 2.0)
        print("t = ", t)
        print("p = ", p)
        print("q = ", q)
        print("l = ", t + p + q)

        touch_pt1 = rotate_point([0, 0], c1, -t)
        arc1 = get_arc(c1, touch_pt1, t)
        c3 = 2 * touch_pt1 - c1
        arc2 = get_arc(c3, touch_pt1, p)
        arc3 = get_arc(c2, [d, 0], q)

        plt.scatter([c1[0], c2[0], c3[0]], [c1[1], c2[1], c3[1]], color="k")
        plt.plot(*arc1, "k")
        plt.plot(*arc2, "k")
        plt.plot(*arc3, "k")

        p = mod2pi(2 * np.pi - p)
        t = mod2pi(alpha - theta + p / 2.0)
        q = mod2pi(alpha - beta - t + p)
        print("t = ", t)
        print("p = ", p)
        print("q = ", q)
        print("l = ", t + p + q)

        touch_pt1 = rotate_point([0, 0], c1, -t)
        arc1 = get_arc(c1, touch_pt1, t)
        c3 = 2 * touch_pt1 - c1
        arc2 = get_arc(c3, touch_pt1, p)
        arc3 = get_arc(c2, [d, 0], q)

        plt.plot(*arc1, "g")
        plt.plot(*arc2, "g")
        plt.plot(*arc3, "g")

        plt.plot(*get_circle(c1, 1.0), "b--")
        plt.plot(*get_circle(c2, 1.0), "r--")
        plt.quiver(0, 0, 2 * ca, 2 * sa, color="b")
        plt.quiver(d, 0, 2 * cb, 2 * sb, color="r")
        plt.scatter([0, d], [0, 0], color="r")
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    unittest.main()
