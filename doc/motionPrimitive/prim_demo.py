"""
Author: Jordan Ramsdell
Desc: Demonstrates how to use read_primitives and Primitive
Usage: python3 prim_demo.py primitives.txt
"""
from primutils import Primitive, read_primitives
import math
import sys


if __name__ == '__main__':
    # Reads the primitives.txt file given as the first argument
    # ... and returns a list of Primitive objects
    primitives = read_primitives(sys.argv[1])
    print(primitives)
    p = primitives[0] # we're looking at the first primitive generated

    # angular acceleration and linear acceleration are stored in each primitive
    print("Angular / Linear Accel: {}/{}\n".format(p.wa, p.va))

    # the divisors are also stored (used in generating keys)
    print("Divisors: {}\n".format(p.config))

    # entries is a dictionary shared between Primitives,
    # where we store the precalculated applications of primitives
    print("# of precalculated entries: {}\n".format(len(p.entries)))

    x = 4
    y = 4
    linear_velocity = 0.3
    # angular_velocity = math.pi / 5
    heading = math.pi / 2

    # here's an example of getting a precalculated result of applying the primitive
    print("Precalculated Entry: {}\n".format(
        p.get_entry(linear_velocity, heading)))

    # Invalid states have no entries. For instance, we can't go faster than 1.2
    print("Precalculated Entry: {}\n".format(
        p.get_entry(2, heading)))

    # Finally, you can calculate results on the fly by applying the primitive:
    print("New Result: {}\n".format(
        p.apply(x, y, linear_velocity, math.pi / 4, heading)))



