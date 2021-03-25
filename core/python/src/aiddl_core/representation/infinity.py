import aiddl_core.representation.term as term
import aiddl_core.representation.numerical as numerical
import aiddl_core.representation.integer as integer


class Infinity(numerical.Numerical):
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
            raise AttributeError("Not a number: "
                                 + str(self) + " + "
                                 + str(other))
        elif isinstance(other, numerical.Numerical):
            return self
        raise AttributeError("Not a numerical term: " + str(other))

    def __sub__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive != other._is_positive:
                return self
            raise AttributeError("Not a number: "
                                 + str(self) + " - "
                                 + str(other))
        elif isinstance(other, numerical.Numerical):
            return self
        raise AttributeError("Not a numerical term: " + str(other))

    def __mul__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive == other._is_positive:
                return Infinity.pos()
            else:
                return Infinity.neg()
        elif isinstance(other, numerical.Numerical):
            if other >= integer.Integer(0) and \
               other < integer.Integer(1):
                return integer.Integer(0)
            else:
                if self._is_positive == (other > integer.Integer(0)):
                    return Infinity.pos()
                else:
                    return Infinity.neg()
        raise AttributeError("Not a numerical term: " + str(other))

    def __truediv__(self, other):
        if isinstance(other, Infinity):
            raise AttributeError("Not a number: "
                                 + str(self) + " / "
                                 + str(other))
        elif isinstance(other, numerical.Numerical):
            if self._is_positive == (other > integer.Integer(0)):
                return Infinity.pos()
            else:
                return Infinity.neg()
        raise AttributeError("Not a numerical term: " + str(other))

    def __floordiv__(self, other):
        if isinstance(other, Infinity):
            raise AttributeError("Not a number: "
                                 + str(self) + " // "
                                 + str(other))
        elif isinstance(other, numerical.Numerical):
            if self._is_positive == (other > integer.Integer(0)):
                return Infinity.pos()
            else:
                return Infinity.neg()
        raise AttributeError("Not a numerical term: " + str(other))

    def __str__(self):
        if self._is_positive:
            return "+INF"
        else:
            return "-INF"

    def __eq__(self, other):
        if isinstance(other, Infinity):
            return self._is_positive == other._is_positive
        elif isinstance(other, numerical.Numerical):
            return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Infinity):
            if not self._is_positive and other._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Numerical):
            return not self._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __le__(self, other):
        if isinstance(other, Infinity):
            if not self._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Numerical):
            return not self._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __gt__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive and not other._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Numerical):
            return self._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __ge__(self, other):
        if isinstance(other, Infinity):
            if self._is_positive:
                return True
            else:
                return False
        if isinstance(other, numerical.Numerical):
            return self._is_positive
        raise AttributeError("Not a numerical term: " + str(other))

    def __hash__(self):
        if self.pos:
            return 17*hash("+INF")
        else:
            return 17*hash("-INF")

    def unpack(self):
        return str(self)
