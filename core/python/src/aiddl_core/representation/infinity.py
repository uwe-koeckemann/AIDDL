import aiddl_core.representation.term as term
import aiddl_core.representation.num as numerical
import aiddl_core.representation.int as integer
import aiddl_core.representation.nan as nan


class Infinity(numerical.Num):
    __slots__ = ["_is_positive"]

    @staticmethod
    def pos():
        return Infinity(True)

    @staticmethod
    def neg():
        return Infinity(False)

    def __init__(self, is_positive):
        super(term.Term, self).__setattr__("_is_positive", is_positive)

    def resolve(self, container):
        return self

    def __add__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive == other._is_positive:
                return self
            return nan.NaN()
        elif isinstance(other, numerical.Num):
            return self
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive != other._is_positive:
                return self
            return nan.NaN()
        elif isinstance(other, numerical.Num):
            return self
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive == other._is_positive:
                return Infinity.pos()
            else:
                return Infinity.neg()
        elif isinstance(other, numerical.Num):
            if other == integer.Int(0):
                return nan.NaN()
            else:
                if self._is_positive == (other > integer.Int(0)):
                    return Infinity.pos()
                else:
                    return Infinity.neg()
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Infinity):
            return nan.NaN()
        elif isinstance(other, numerical.Num):
            if self._is_positive == (other > integer.Int(0)):
                return Infinity.pos()
            else:
                return Infinity.neg()
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Infinity):
            return nan.NaN()
        elif isinstance(other, numerical.Num):
            if self._is_positive == (other > integer.Int(0)):
                return Infinity.pos()
            else:
                return Infinity.neg()
        elif isinstance(other, nan.NaN):
            return other
        raise AttributeError("Not a numerical term: " + str(other))

    def __str__(self):
        if self._is_positive:
            return "+INF"
        else:
            return "-INF"

    def __eq__(self, other):
        return False

    def __ne__(self, other):
        return True

    def __lt__(self, other):
        if isinstance(other, Infinity):
            if not self._is_positive and other.is_positive():
                return True
            else:
                return False
        if isinstance(other, numerical.Num):
            return not self._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Infinity):
            if not self._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Num):
            return not self._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive and not other.is_positive():
                return True
            else:
                return False
        if isinstance(other, numerical.Num):
            return self._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Num):
            return self._is_positive
        elif isinstance(other, nan.Nan):
            return False
        raise AttributeError("Not a numerical term: " + str(other))

    def is_zero(self):
        return False

    def is_positive(self):
        return self._is_positive

    def is_negative(self):
        return not self._is_positive

    def is_inf(self):
        return True

    def is_inf_pos(self):
        return self._is_positive

    def is_inf_neg(self):
        return not self._is_positive

    def __hash__(self):
        if self.pos:
            return 17*hash("+INF")
        else:
            return 17*hash("-INF")

    def unpack(self):
        return str(self)
