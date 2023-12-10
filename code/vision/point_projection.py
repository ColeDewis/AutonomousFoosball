"""Code from PyRival for getting closest point on a line to another point off
the line (project point to line)

https://github.com/cheran-senthil/PyRival/blob/master/pyrival/geometry/vectors.py
"""

to_vec = lambda p1, p2: (j - i for i, j in zip(p1, p2))

scale = lambda v, s: (i * s for i in v)

translate = lambda p, v: (pi + vi for pi, vi in zip(p, v))

dot = lambda v1, v2: sum(i * j for i, j in zip(v1, v2))

norm_sq = lambda v: sum(i * i for i in v)

def closest_point(p, a, b, segment=False):
    ap, ab = to_vec(a, p), to_vec(a, b)
    abb = to_vec(a, b)
    abbb = to_vec(a, b)

    u = dot(ap, ab) / norm_sq(abb)

    if segment:
        if u < 0:
            return a
        if u > 1:
            return b

    return translate(a, scale(abbb, u))
