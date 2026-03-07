import numpy


def func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d):
    return (
        0.0002349228528 * f_d * numpy.cos(theta)
        + 0.0003697488 * phi_dot * numpy.cos(theta)
        + 0.00020119778 * phi_dot
        - 0.003697488 * tau * numpy.cos(theta)
        - 0.0040239556 * tau
        - 6.835708755072e-6 * theta_dot**2 * numpy.sin(2 * theta)
        + 0.0001848744 * theta_dot * numpy.cos(theta)
        + 0.00270200948397752 * numpy.sin(theta)
    ) / (1.3671417510144e-5 * numpy.sin(theta) ** 2 + 2.91333618973888e-5)
