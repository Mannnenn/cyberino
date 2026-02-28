import numpy


def func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d):
    return (
        -0.01125 * f_d * numpy.cos(theta)
        - 0.0135 * f_d * numpy.cos(2 * theta)
        + 0.0045 * f_d
        - 0.00675 * phi_dot * numpy.cos(theta)
        - 0.0193125 * phi_dot
        + 0.09 * tau * numpy.cos(theta)
        + 0.20625 * tau
        + 0.0081 * theta_dot**2 * numpy.sin(theta)
        + 0.0010125 * theta_dot**2 * numpy.sin(2 * theta)
        - 0.00225 * theta_dot * numpy.cos(theta)
        - 0.009 * theta_dot
        - 0.115841053125 * numpy.sin(theta)
        - 0.09929233125 * numpy.sin(2 * theta)
    ) / (0.002025 * numpy.sin(theta) ** 2 + 0.0027)
