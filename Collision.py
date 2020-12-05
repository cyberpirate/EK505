
def sphereInter(pos, fwd, spos, r):

    # A B

    # A + dot(AP,AB) / dot(AB,AB) * AB
    projPoint = pos + (spos-pos).dot(fwd) / fwd.dot(fwd) * fwd

    return (projPoint-spos).mag <= r