import numpy


def func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d):
    return (
        0.01125 * f_d * numpy.cos(theta)
        + 0.0045 * phi_dot * numpy.cos(theta)
        + 0.0013125 * phi_dot
        - 0.045 * tau * numpy.cos(theta)
        - 0.02625 * tau
        - 0.0010125 * theta_dot**2 * numpy.sin(2 * theta)
        + 0.00225 * theta_dot * numpy.cos(theta)
        + 0.115841053125 * numpy.sin(theta)
    ) / (0.002025 * numpy.sin(theta) ** 2 + 0.0027)
