"""Waterline integration helpers for line-based members."""


def segment_air_fraction(z1, z2, waterline, transition_width=0.0):
    """Length-average air fraction along a straight segment.

    With a zero-width transition this is the exact portion above the waterline.
    A positive width applies the input's linear waterline transition and
    integrates that fraction exactly along the segment.
    """
    low, high = sorted((float(z1), float(z2)))
    width = float(transition_width)
    if width <= 0.0:
        if high <= waterline:
            return 0.0
        if low >= waterline:
            return 1.0
        return (high - waterline) / (high - low)
    if high == low:
        return max(0.0, min(1.0, (low - waterline + width / 2.0) / width))

    bottom = waterline - width / 2.0
    top = waterline + width / 2.0

    def primitive(z):
        if z <= bottom:
            return 0.0
        if z >= top:
            return z - waterline
        offset = z - bottom
        return offset * offset / (2.0 * width)

    return max(0.0, min(1.0, (primitive(high) - primitive(low)) / (high - low)))
