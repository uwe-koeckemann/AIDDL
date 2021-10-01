import math

from aiddl_core.representation.int import Int
from aiddl_core.representation.rat import Rat
from aiddl_core.representation.real import Real


class ExptFunction:
    def __call__(self, x):
        base = x[0]
        exp = x[1]
        if isinstance(base, Real) \
           or isinstance(exp, Real) \
           or isinstance(exp, Rat):
            return Real(base.real_value() ** exp.real_value())
        else:
            r = Int(1)
            for i in range(exp.int_value()):
                r *= base
            return r



