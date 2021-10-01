import aiddl_core.representation.term as term
import aiddl_core.representation.num as numerical
import aiddl_core.representation.rat as rational
import aiddl_core.representation.real as real
import aiddl_core.representation.infinity as infinity
import aiddl_core.representation.nan as nan

from math import floor


class Int(numerical.Num):
    __slots__ = ["_value"]

    def __init__(self, value):
        super(term.Term, self).__setattr__("_value", value)

    def resolve(self, container):
        return self

    def int_value(self):
        return self._value

    def real_value(self):
        return float(self._value)

    def __add__(self, other):
        if isinstance(other, Int):
            return Int(self._value + other._value)
        elif isinstance(other, rational.Rat):
            return rational.Rat(self._value * other._q + other._p, other._q)
        elif isinstance(other, real.Real):
            return real.Real(self._value + other._value)
        elif isinstance(other, infinity.Infinity):
            return other
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other) + " type: " + str(type(other)) )

    def __sub__(self, other):
        if isinstance(other, Int):
            return Int(self._value - other._value)
        elif isinstance(other, rational.Rat):
            return rational.Rat(self._value * other._q - other._p,
                                other._q)
        elif isinstance(other, real.Real):
            return real.Real(self._value - other._value)
        elif isinstance(other, infinity.Infinity):
            return infinity.Infinity(not other._is_positive)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Int):
            return Int(self._value * other._value)
        elif isinstance(other, rational.Rat):
            return rational.Rat(self._value * other._p, other._q)
        elif isinstance(other, real.Real):
            return real.Real(self._value * other._value)
        elif isinstance(other, infinity.Infinity):
            if self._value == 0:
                return nan.NaN()
            else:
                return infinity.Infinity(other._is_positive ==
                                              (self._value > 0))
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, numerical.Num) and other.is_zero():
            return nan.NaN()
        if isinstance(other, Int):
            return real.Real(self._value / other._value)
        elif isinstance(other, rational.Rat):
            return rational.Rat(self._value * other._q, other._p)
        elif isinstance(other, real.Real):
            return rational.Rat(self._value, other._value)
        elif isinstance(other, infinity.Infinity):
            return Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, numerical.Num) and other.is_zero():
            return nan.NaN()
        if isinstance(other, Int):
            return Int(self._value // other._value)
        elif isinstance(other, rational.Rat):
            return Int((self._value * other._q) // other._p)
        elif isinstance(other, real.Real):
            return Int(int(floor(self._value / other._value)))
        elif isinstance(other, infinity.Infinity):
            return Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __str__(self):
        return str(self._value)

    def __repr__(self):
        return str(self._value)

    def __eq__(self, other):
        if isinstance(other, Int):
            return self._value == other._value
        elif isinstance(other, rational.Rat):
            return self._value*other._q == other._p
        elif isinstance(other, real.Real):
            return float(self._value) == other._value
        elif isinstance(other, infinity.Infinity):
            return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Int):
            return self._value < other._value
        elif isinstance(other, rational.Rat):
            return self._value*other._q < other._p
        elif isinstance(other, real.Real):
            return float(self._value) < other._value
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Int):
            return self._value <= other._value
        elif isinstance(other, rational.Rat):
            return self._value*other._q <= other._p
        elif isinstance(other, real.Real):
            return float(self._value) <= other._value
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Int):
            return self._value > other._value
        elif isinstance(other, rational.Rat):
            return self._value*other._q > other._p
        elif isinstance(other, real.Real):
            return float(self._value) > other._value
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Int):
            return self._value >= other._value
        elif isinstance(other, rational.Rat):
            return self._value*other._q >= other._p
        elif isinstance(other, real.Real):
            return float(self._value) >= other._value
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def is_zero(self):
        return self._value == 0

    def is_positive(self):
        return self._value > 0

    def is_negative(self):
        return self._value < 0

    def __hash__(self):
        return hash(self._value)

    def unpack(self):
        return self._value
