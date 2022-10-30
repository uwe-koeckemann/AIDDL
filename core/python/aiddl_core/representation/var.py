import aiddl_core.representation.term as term
from aiddl_core.representation.substitution import Substitution


class Var(term.Term):
    __slots__ = ["_name", "_is_anonymous"]
    __anon_count = 0

    def __init__(self, name=None):
        if name is None:
            Var.__anon_count += 1
            name = "?_X%d" % (Var.__anon_count)
            is_anon = True
        else:
            if name[0] != "?":
                name = "?" + name
            is_anon = False
        super(term.Term, self).__setattr__("_name", name)
        super(term.Term, self).__setattr__("_is_anonymous", is_anon)

    @property
    def is_anonymous(self):
        return self._is_anonymous

    def match(self, other):
        s = Substitution()
        s.add(self, other)
        return s

    def unpack(self):
        return str(self)

    def __str__(self):
        if self._is_anonymous:
            return "_"
        else:
            return str(self._name)

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return 5*hash(self._name)

    def __eq__(self, other):
        if isinstance(other, Var):
            return self._name == other._name 
        else:
            return False

    def __ne__(self, other):
        return not self == other
