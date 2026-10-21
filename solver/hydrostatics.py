"""Waterline integration helpers for line-based members."""

import math


def segment_air_fraction(z1, z2, waterline, transition_width=0.0, circular_section=False):
    """Length-average air fraction along a straight segment.

    With a zero-width transition this is the exact portion above the waterline.
    A positive width applies either a linear transition or the submerged area
    fraction of a circular section, then averages it along the segment.
    """
    low, high = sorted((float(z1), float(z2)))
    width = float(transition_width)
    if width <= 0.0:
        if high <= waterline:
            return 0.0
        if low >= waterline:
            return 1.0
        return (high - waterline) / (high - low)
    if high == low and not circular_section:
        return max(0.0, min(1.0, (low - waterline + width / 2.0) / width))

    if circular_section:
        def air_fraction_at(z):
            u = max(-1.0, min(1.0, (z - waterline) / (width / 2.0)))
            return (math.asin(u) + math.pi / 2.0
                    + u * math.sqrt(max(0.0, 1.0 - u * u))) / math.pi

        if high == low:
            return air_fraction_at(low)

    bottom = waterline - width / 2.0
    top = waterline + width / 2.0

    if circular_section:
        # Fixed Gauss quadrature integrates the section immersion smoothly
        # along a tilted segment, including segments that cross the surface.
        import numpy as np

        points, weights = np.polynomial.legendre.leggauss(16)
        fractions = [air_fraction_at((low + high) / 2.0 + (high - low) * point / 2.0)
                     for point in points]
        return float(sum(weight * fraction for weight, fraction in zip(weights, fractions)) / 2.0)

    def primitive(z):
        if z <= bottom:
            return 0.0
        if z >= top:
            return z - waterline
        offset = z - bottom
        return offset * offset / (2.0 * width)

    return max(0.0, min(1.0, (primitive(high) - primitive(low)) / (high - low)))
