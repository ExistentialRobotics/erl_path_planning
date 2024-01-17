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
    return np.array([x, y])


def get_line(start, direction, length, n=50):
    t = np.linspace(0, length, n)
    x = start[0] + direction[0] * t
    y = start[1] + direction[1] * t
    return np.array([x, y])


def mod2pi(theta):
    theta = np.fmod(theta, 2 * np.pi)
    if theta < -np.pi:
        theta += 2 * np.pi
    elif theta >= np.pi:
        theta -= 2 * np.pi
    return theta


class VisualizeReedsShepp(unittest.TestCase):
    def test_lrsl(self):
        x = 3
        y = 4
        phi = np.deg2rad(120)

        xi = x - np.sin(phi)
        eta = y - 1.0 + np.cos(phi)
        rho = np.sqrt(xi * xi + eta * eta)
        theta = np.arctan2(eta, xi)
        assert rho > 2.0

        r = np.sqrt(rho * rho - 4.0)
        u = 2 - r
        t = mod2pi(theta + np.arctan2(r, -2))
        # t = mod2pi(theta + np.arctan2(r, 2))
        v = mod2pi(phi - np.pi / 2 - t)

        print("t = ", t)
        print("u = ", u)
        print("v = ", v)

        c1 = np.array([0, 1])
        arc1 = get_arc(c1, [0, 0], t)
        p1 = rotate_point([0, 0], [0, 1], t)
        c2 = 2 * p1 - c1
        arc2 = get_arc(c2, p1, np.pi / 2)
        p2 = rotate_point(p1, c2, np.pi / 2)
        d1 = np.array([-np.sin(t), np.cos(t)])
        l2 = get_line(p2, d1, u)
        p3 = p2 + u * d1
        c3 = np.array([xi, eta + 1.0])
        arc3 = get_arc(c3, p3, v)

        plt.quiver(0, 0, 1, 0, color="r")
        plt.quiver(x, y, np.cos(phi), np.sin(phi), color="b")
        plt.quiver(p2[0], p2[1], d1[0], d1[1], color="g")
        plt.plot(*arc1, "k")
        plt.plot(*arc2, "r")
        plt.plot(*l2, "b")
        plt.plot(*arc3, "g")
        plt.plot(*get_circle(c1, 1), "k--")
        plt.plot(*get_circle(c2, 1), "r--")
        plt.plot(*get_circle(c3, 1), "g--")
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    unittest.main()
