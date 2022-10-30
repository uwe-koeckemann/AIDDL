import aiddl_core.representation.term as term
import aiddl_core.representation.num as numerical
import aiddl_core.representation.int as integer
import aiddl_core.representation.real as real
import aiddl_core.representation.inf as infinity
import aiddl_core.representation.nan as nan


from math import floor


class Rat(numerical.Num):
    __slots__ = ["_p", "_q"]

    def __init__(self, n, d):
        if d == 0:
            raise AttributeError("Rational number with zero denominator.")
        gcd = Rat._gcd(n, d)
        super(term.Term, self).__setattr__("_p", n//gcd)
        super(term.Term, self).__setattr__("_q", d//gcd)

    @property
    def nominator(self):
        return self._p

    @property
    def denominator(self):
        return self._q

    @property
    def int_value(self):
        return self._p // self._q

    @property
    def real_value(self):
        return self._p / self._q

    @staticmethod
    def _gcd(a, b):
        if b == 0:
            return a
        else:
            return Rat._gcd(b, a % b)

    def __add__(self, other):
        if isinstance(other, Rat):
            return Rat(self._p * other._q + other._p * self._q,
                       self._q * other._q)
        elif isinstance(other, integer.Int):
            return Rat(self._p + self._q * other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q + other._value)
        elif isinstance(other, infinity.Inf):
            return other
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Rat):
            return Rat(self._p * other._q - other._p * self._q,
                       self._q * other._q)
        elif isinstance(other, integer.Int):
            return Rat(self._p - self._q * other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q - other._value)
        elif isinstance(other, infinity.Inf):
            return infinity.Inf(not other._is_positive)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Rat):
            return Rat(self._p * other._p, self._q * other._q)
        elif isinstance(other, integer.Int):
            return Rat(self._p * other._value, self._q)
        elif isinstance(other, real.Real):
            return real.Real(self._p / self._q * other._value)
        elif isinstance(other, infinity.Inf):
            if self == integer.Int(0):
                return nan.NaN()
            else:
                return infinity.Inf(other._is_positive ==
                                    (self > integer.Int(0)))
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Rat):
            return Rat(self._p * other._q, self._q * other._p)
        elif isinstance(other, integer.Int):
            return Rat(self._p, self._q * other._value)
        elif isinstance(other, real.Real):
            return real.Real((self._p / self._q) / other._value)
        elif isinstance(other, infinity.Inf):
            return integer.Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Rat):
            return integer.Int((self._p * other._q)
                               // (self._q * other._p))
        elif isinstance(other, integer.Int):
            return integer.Int(self._p // (self._q * other._value))
        elif isinstance(other, real.Real):
            return integer.Int(int(floor(self._p / self._q)
                                   / other._value))
        elif isinstance(other, infinity.Inf):
            return integer.Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __str__(self):
        return str(self._p)+"/"+str(self._q)

    def __eq__(self, other):
        if isinstance(other, Rat):
            return self._p*other._q == other._p*self._q
        elif isinstance(other, integer.Int):
            return self._p == other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) == other._value
        elif isinstance(other, infinity.Inf):
            return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Rat):
            return self._p*other._q < other._p*self._q
        elif isinstance(other, integer.Int):
            return self._p < other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) < other._value
        elif isinstance(other, infinity.Inf):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Rat):
            return self._p*other._q <= other._p*self._q
        elif isinstance(other, integer.Int):
            return self._p <= other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) <= other._value
        elif isinstance(other, infinity.Inf):
            return other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Rat):
            return self._p*other._q > other._p*self._q
        elif isinstance(other, integer.Int):
            return self._p > other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) > other._value
        elif isinstance(other, infinity.Inf):
            return not other._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Rat):
            return self._p*other._q >= other._p*self._q
        elif isinstance(other, integer.Int):
            return self._p >= other._value*self._q
        elif isinstance(other, real.Real):
            return (self._p / self._q) >= other._value
        elif isinstance(other, infinity.Inf):
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
