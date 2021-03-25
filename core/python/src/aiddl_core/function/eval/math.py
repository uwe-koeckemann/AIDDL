import math

from aiddl_core.representation.integer import Integer
from aiddl_core.representation.rational import Rational
from aiddl_core.representation.real import Real
from aiddl_core.representation.symbolic import Boolean


class CosineFunction:
    def apply(self, x):
        return Real(math.cos(x.real_value()))


class SineFunction:
    def apply(self, x):
        return Real(math.sin(x.real_value()))


class ArcCosineFunction:
    def apply(self, x):
        return Real(math.acos(x.real_value()))


class ArcSineFunction:
    def apply(self, x):
        return Real(math.asin(x.real_value()))


class TangentFunction:
    def apply(self, x):
        return Real(math.tan(x.real_value()))


class ArcTangentFunction:
    def apply(self, x):
        return Real(math.atan(x.real_value()))


class SqrtFunction:
    def apply(self, x):
        return Real(math.sqrt(x.real_value()))


class ExptFunction:
    def apply(self, x):
        base = x[0]
        exp = x[1]
        if isinstance(base, Real) \
           or isinstance(exp, Real) \
           or isinstance(exp, Rational):
            return Real(base.real_value() ** exp.real_value())
        else:
            r = Integer(1)
            for i in range(exp.int_value()):
                r *= base
            return r


class LogFunction:
    def apply(self, x):
        return Real(math.log(x.real_value()))


class LnFunction:
    def apply(self, x):
        return Real(math.log10(x.real_value()))


class Log2Function:
    def apply(self, x):
        return Real(math.log2(x.real_value()))
