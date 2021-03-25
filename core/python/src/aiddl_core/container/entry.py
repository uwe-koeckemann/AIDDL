from aiddl_core.representation.term import Term
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.reference import Reference
from aiddl_core.representation.tuple import Tuple

class Entry(Term):
    __slots__ = ["_type", "_name", "_value"]

    def __init__(self, entry_type, entry_name, entry_value):
        if entry_type is None or entry_name is None or entry_value is None:
            raise ValueError("Entry constructor needs non-null arguments.")
        super(Term, self).__setattr__("_type", entry_type)
        super(Term, self).__setattr__("_name", entry_name)
        super(Term, self).__setattr__("_value", entry_value)

    def get_type(self):
        return self._type

    def get_name(self):
        return self._name

    def get_value(self):
        return self._value

    def as_tuple(self):
        return Tuple([self._type, self._name, self._value])

    def substitute(self, s):
        return Entry(self._type.substitute(s),
                     self._name.substitute(s),
                     self._value.substitute(s))

    def __hash__(self):
        return self._type.__hash__() \
            + 3*self._name.__hash__() \
            + 7*self._value.__hash__()

    def __eq__(self, other):
        if isinstance(other, Entry):
            return other._type == self._type \
                and other._name == self._name \
                and other._value == self._value
        return False

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        s = ""
        s += "("
        s += str(self._type)
        s += " "
        s += str(self._name)
        s += " "
        s += str(self._value)
        s += ")"
        return s

    def __repr__(self):
        return str(self._value)

    def unpack(self):
        return (self._type.unpack(),
                self._name.unpack(),
                self._value.unpack())
