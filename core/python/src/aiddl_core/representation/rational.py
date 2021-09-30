import aiddl_core.representation.term as term
import aiddl_core.representation.numerical as numerical
import aiddl_core.representation.integer as integer
import aiddl_core.representation.real as real
import aiddl_core.representation.infinity as infinity
import aiddl_core.representation.nan as nan


from aiddl_core.tools.profiler import Profiler

from math import floor


class Rational(numerical.Numerical):
    __slots__ = ["_p", "_q"]

    def __init__(self, n, d):
        if d == 0:
            raise AttributeError("Rational number with zero denominator.")
        gcd = Rational.gcd(n, d)
        super(term.Term, self).__setattr__("_p", n//gcd)
        super(term.Term, self).__setattr__("_q", d//gcd)

    def resolve(self, container):
        return self

    def int_value(self):
        return self._p // self._q

    def real_value(self):
        return self._p / self._q


    @staticmethod
    def gcd(a, b):
        if b == 0:
            return a
        else:
            return Rational.gcd(b, a % b)

    def __add__(self, other):
        if isinstance(other, Rational):
            return Rational(self._p * other._q + other._p * self._q,
                            self._q * other._q)
        elif isinstance(other, integer.Integer):
            return Rational(self._p + self._q*other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q + other._value)
        elif isinstance(other, infinity.Infinity):
            return other
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Rational):
            return Rational(self._p*other._q - other._p*self._q,
                            self._q * other._q)
        elif isinstance(other, integer.Integer):
            return Rational(self._p - self._q*other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q - other._value)
        elif isinstance(other, infinity.Infinity):
            return infinity.Infinity(not other._is_positive)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Rational):
            return Rational(self._p * other._p, self._q * other._q)
        elif isinstance(other, integer.Integer):
            return Rational(self._p * other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q * other._value)
        elif isinstance(other, infinity.Infinity):
            if self == integer.Integer(0):
                return nan.NaN()
            else:
                return infinity.Infinity(other._is_positive ==
                                              (self > integer.Integer(0)))
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Rational):
            return Rational(self._p * other._q, self._q * other._p)
        elif isinstance(other, integer.Integer):
            return Rational(self._p, self._q * other._value)
        elif isinstance(other, real.Real):
            return real.Real((self._p / self._q) / other._value)
        elif isinstance(other, infinity.Infinity):
            return integer.Integer(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Rational):
            return integer.Integer((self._p * other._q)
                                        // (self._q * other._p))
        elif isinstance(other, integer.Integer):
            return integer.Integer(self._p // (self._q * other._value))
        elif isinstance(other, real.Real):
            return integer.Integer(int(floor(self._p / self._q)
                                            / other._value))
        elif isinstance(other, infinity.Infinity):
            return integer.Integer(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __str__(self):
        return str(self._p)+"/"+str(self._q)

    def __eq__(self, other):
        if isinstance(other, Rational):
            return self._p*other._q == other._p*self._q
        elif isinstance(other, integer.Integer):
            return self._p == other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) == other._value
        elif isinstance(other, infinity.Infinity):
            return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Rational):
            return self._p*other._q < other._p*self._q
        elif isinstance(other, integer.Integer):
            return self._p < other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) < other._value
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Rational):
            return self._p*other._q <= other._p*self._q
        elif isinstance(other, integer.Integer):
            return self._p <= other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) <= other._value
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Rational):
            return self._p*other._q > other._p*self._q
        elif isinstance(other, integer.Integer):
            return self._p > other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) > other._value
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Rational):
            return self._p*other._q >= other._p*self._q
        elif isinstance(other, integer.Integer):
            return self._p >= other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) >= other._value
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def is_zero(self):
        return self._p == 0

    def is_positive(self):
        return self._p > 0

    def is_negative(self):
        return self._p < 0

    def __hash__(self):
        return hash(self._p) + hash(self._q)

    def unpack(self):
        return str(self._p) + "/" + str(self._q)
