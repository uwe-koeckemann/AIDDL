import aiddl_core.representation.term as term


class Sym(term.Term):
    __slots__ = ["_value"]

    def __init__(self, value):
        # if value == "+INF":
        #     raise ValueError("This should be infinity")
        if not isinstance(value, str):
            raise ValueError("Symbolic must be instanciated with a string.")
        super(term.Term, self).__setattr__("_value", value)

    def substitute(self, substitution):
        return substitution.substitute(self)

    def resolve(self, container):
        return self

    def string_value(self):
        return self._value

    def bool_value(self):
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
    __slots__ = ["_bool_value"]

    def __init__(self, value):
        if value:
            super(term.Term, self).__setattr__("_value", str(True))
            super(term.Term, self).__setattr__("_bool_value", True)
        else:
            super(term.Term, self).__setattr__("_value", str(False))
            super(term.Term, self).__setattr__("_bool_value", False)

    def bool_value(self):
        return self._bool_value

    @staticmethod
    def create(val):
        if val:
            return TRUE
        else:
            return FALSE

    def __not__(self):
        if self._bool_value:
            return FALSE
        else:
            return TRUE

    def __or__(self, other):
        return Boolean.create(self._bool_value or other._bool_value)

    def __and__(self, other):
        return Boolean.create(self._bool_value and other._bool_value)

    def unpack(self):
        return self._bool_value


TRUE = Boolean(True)
FALSE = Boolean(False)


a = Sym("a")
b = Sym("b")

assert(a == a)
assert(not (a == b))
assert(a != b)
