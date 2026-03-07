import numpy


def func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d):
    return (
        -0.0002349228528 * f_d * numpy.cos(theta)
        - 0.000199664352 * f_d * numpy.cos(2 * theta)
        + 0.00037476 * f_d
        - 0.0005546232 * phi_dot * numpy.cos(theta)
        - 0.00126494658 * phi_dot
        + 0.007394976 * tau * numpy.cos(theta)
        + 0.0146614436 * tau
        + 3.9331984230144e-5 * theta_dot**2 * numpy.sin(theta)
        + 6.835708755072e-6 * theta_dot**2 * numpy.sin(2 * theta)
        - 0.0001848744 * theta_dot * numpy.cos(theta)
        - 0.0005318744 * theta_dot
        - 0.00270200948397752 * numpy.sin(theta)
        - 0.00124139635672087 * numpy.sin(2 * theta)
    ) / (1.3671417510144e-5 * numpy.sin(theta) ** 2 + 2.91333618973888e-5)
