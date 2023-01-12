import aiddl_core.representation.term as term
import aiddl_core.representation.num as numerical
import aiddl_core.representation.int as integer
import aiddl_core.representation.rat as rational
import aiddl_core.representation.inf as infinity
import aiddl_core.representation.nan as nan

from math import floor


class Real(numerical.Num):
    __slots__ = ["_value"]

    def __init__(self, value):
        super(term.Term, self).__setattr__("_value", value)

    @property
    def real_value(self):
        """ Return this integer as a python float object

        :return: float value
        """
        return self._value

    @property
    def int_value(self):
        """ Return this integer as a python int object

        :return: int value
        """
        return int(self._value)

    def __add__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return Real(self._value + other._value)
        elif isinstance(other, rational.Rat):
            return Real(self._value + float(other._p)/float(other._q))
        elif isinstance(other, infinity.Inf):
            return other
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return Real(self._value - other._value)
        elif isinstance(other, rational.Rat):
            return Real(self._value - float(other._p)/float(other._q))
        elif isinstance(other, infinity.Inf):
            return infinity.Inf(not other._is_positive)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return Real(self._value * other._value)
        elif isinstance(other, rational.Rat):
            return Real(self._value * float(other._p)/float(other._q))
        elif isinstance(other, infinity.Inf):
            if self == integer.Int(0):
                return nan.NaN()
            else:
                return infinity.Inf(other._is_positive
                                    == (self._value > 0))
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return Real(self._value / other._value)
        elif isinstance(other, rational.Rat):
            return Real(self._value / (other._p/other._q))
        elif isinstance(other, infinity.Inf):
            return integer.Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return integer.Int(floor(self._value / other._value))
        elif isinstance(other, rational.Rat):
            return integer.Int(int(floor(self._value /
                                         other._p /
                                         other._q)))
        elif isinstance(other, infinity.Inf):
            return integer.Int(0)
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __repr__(self):
        return str(self._value)
    
    def __str__(self):
        return str(self._value)

    def __eq__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return self._value == other._value
        elif isinstance(other, rational.Rat):
            return self._value == float(other._p)/float(other._q)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return self._value < other._value
        elif isinstance(other, rational.Rat):
            return self._value < float(other._p)/float(other._q)
        elif isinstance(other, infinity.Inf):
            return other._is_positive
        elif isinstance(other, nan.NaN):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return self._value <= other._value
        elif isinstance(other, rational.Rat):
            return self._value <= float(other._p)/float(other._q)
        elif isinstance(other, infinity.Inf):
            return other._is_positive
        elif isinstance(other, nan.NaN):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return self._value > other._value
        elif isinstance(other, rational.Rat):
            return self._value > float(other._p)/float(other._q)
        elif isinstance(other, infinity.Inf):
            return not other._is_positive
        elif isinstance(other, nan.NaN):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Real) or isinstance(other, integer.Int):
            return self._value >= other._value
        elif isinstance(other, rational.Rat):
            return self._value >= float(other._p)/float(other._q)
        elif isinstance(other, infinity.Inf):
            return not other._is_positive
        elif isinstance(other, nan.NaN):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def is_zero(self):
        return self._value == 0.0

    def is_positive(self):
        return self._value > 0.0

    def is_negative(self):
        return self._value < 0.0

    def __hash__(self):
        return 3*hash(self._value)

    def unpack(self):
        return self._value
