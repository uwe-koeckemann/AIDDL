import aiddl_core.representation
from aiddl_core.representation import term


class Sym(term.Term):
    __slots__ = ["_value"]

    def __init__(self, value):
        # if value == "+INF":
        #     raise ValueError("This should be infinity")
        if not isinstance(value, str):
            raise ValueError("Symbolic must be instanciated with a string.")
        super(term.Term, self).__setattr__("_value", value)

    @property
    def string(self) -> str:
        """ Get the string of this value

        :return: string value of this symbol
        """
        return self._value

    @property
    def bool(self) -> bool:
        """ Get the boolean value of this term if it is true or false

        :return: the boolean value of this term
        """
        if self._value == "true":
            return True
        elif self._value == "false":
            return False
        raise AttributeError("Not a boolean value", self)

    def __add__(self, other):
        if isinstance(other, Sym):
            return Sym(self._value + "." + other._value)
        raise AttributeError("Not a Symbolic: " + str(other)
                             + " (concat only defined between symbolic terms")

    def __str__(self):
        return str(self._value)

    def __repr__(self):
        return str(self._value)

    def __hash__(self):
        return 3*hash(self._value)

    def __eq__(self, other):
        if isinstance(other, Sym):
            return self._value == other._value
        return False

    def __ne__(self, other):
        return not self == other

    def unpack(self):
        return str(self)


class Boolean(Sym):
    __slots__ = ["_bool_value", "_value"]

    def __init__(self, value):
        if value:
            super(term.Term, self).__setattr__("_value", "true")
            super(term.Term, self).__setattr__("_bool_value", True)
        else:
            super(term.Term, self).__setattr__("_value", "false")
            super(term.Term, self).__setattr__("_bool_value", False)

    @property
    def bool(self):
        """ Get the boolean value of this term

        :return: the boolean value
        """
        return self._bool_value

    @staticmethod
    def create(val: bool):
        """ Create a boolean term from a bool object

        :param val: boolean value
        :return: boolean term
        """
        if val:
            return TRUE
        else:
            return FALSE

    def __or__(self, other):
        """ Logical or between this Boolean term and another Boolean term

        :param other: Boolean term
        :return: self or other
        """
        return Boolean.create(self.bool or other.bool)

    def __and__(self, other):
        """ Logical and between this Boolean term and another Boolean term

        :param other: Boolean term
        :return: self and other
        """
        return Boolean.create(self.bool and other.bool)

    def __str__(self):
        return str(self._value)

    def __repr__(self):
        return str(self._value)

    def unpack(self):
        return self._bool_value


TRUE = Boolean(True)
FALSE = Boolean(False)
