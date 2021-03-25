import aiddl_core.representation.term as term
import aiddl_core.representation.numerical as numerical
import aiddl_core.representation.integer as integer
import aiddl_core.representation.rational as rational
import aiddl_core.representation.infinity as infinity

from math import floor


class Real(numerical.Numerical):
    __slots__ = ["_value"]

    def __init__(self, value):
        super(term.Term, self).__setattr__("_value", value)

    def resolve(self, container):
        return self

    def real_value(self):
        return self._value

    def int_value(self):
        return int(self._value)

    def __add__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return Real(self._value + other._value)
        elif isinstance(other, rational.Rational):
            return Real(self._value + float(other._p)/float(other._q))
        elif isinstance(other, infinity.Infinity):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return Real(self._value - other._value)
        elif isinstance(other, rational.Rational):
            return Real(self._value - float(other._p)/float(other._q))
        elif isinstance(other, infinity.Infinity):
            return infinity.Infinity(not other._is_positive)
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return Real(self._value * other._value)
        elif isinstance(other, rational.Rational):
            return Real(self._value * float(other._p)/float(other._q))
        elif isinstance(other, infinity.Infinity):
            if self >= integer.Integer(0) \
               and self < integer.Integer(1):
                return integer.Integer(0)
            else:
                return infinity.Infinity(other._is_positive
                                              == (self._value > 0))
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return Real(self._value / other._value)
        elif isinstance(other, rational.Rational):
            return Real(self._value / (other._p/other._q))
        elif isinstance(other, infinity.Infinity):
            return integer.Integer(0)
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return integer.Integer(floor(self._value / other._value))
        elif isinstance(other, rational.Rational):
            return integer.Integer(int(floor(self._value /
                                                  other._p /
                                                  other._q)))
        elif isinstance(other, infinity.Infinity):
            return integer.Integer(0)
        raise AttributeError("Not a numerical term: " + str(other))

    def __repr__(self):
        return str(self._value)
    
    def __str__(self):
        return str(self._value)

    def __eq__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return self._value == other._value
        elif isinstance(other, rational.Rational):
            return self._value == float(other._p)/float(other._q)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return self._value < other._value
        elif isinstance(other, rational.Rational):
            return self._value < float(other._p)/float(other._q)
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return self._value <= other._value
        elif isinstance(other, rational.Rational):
            return self._value <= float(other._p)/float(other._q)
        elif isinstance(other, infinity.Infinity):
            return other._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return self._value > other._value
        elif isinstance(other, rational.Rational):
            return self._value > float(other._p)/float(other._q)
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Integer):
            return self._value >= other._value
        elif isinstance(other, rational.Rational):
            return self._value >= float(other._p)/float(other._q)
        elif isinstance(other, infinity.Infinity):
            return not other._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __hash__(self):
        return 3*hash(self._value)

    def unpack(self):
        return self._value
