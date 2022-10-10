import aiddl_core.representation.term as term


class KeyVal(term.Term):
    __slots__ = ["_key", "_value"]

    def __init__(self, key, value):
        assert isinstance(key, term.Term)
        assert isinstance(value, term.Term)
        super(term.Term, self).__setattr__("_key", key)
        super(term.Term, self).__setattr__("_value", value)

    def substitute(self, s):
        return KeyVal(self._key.substitute(s), self._value.substitute(s))

    def match(self, other):
        if isinstance(other, KeyVal):
            sub_key = self._key.match(other._key)
            if sub_key is None:
                return None
            sub_value = self._value.match(other._value)
            if sub_value is None:
                return None
            if not sub_key.add_substitution(sub_value):
                return None
            return sub_key
        return None

    @property
    def key(self):
        return self._key

    @property
    def value(self):
        return self._value

    def resolve(self, container):
        return KeyVal(self._key.resolve(container),
                      self._value.resolve(container))

    def __repr__(self):
        return str(self._key)+":"+str(self._value)

    def __str__(self):
        return str(self._key)+":"+str(self._value)

    def __eq__(self, other):
        if isinstance(other, KeyVal):
            return self._value == other._value and self._key == other._key
        return False

    def __ne__(self, other):
        return not (self == other)

    def __hash__(self):
        return 19*hash(self._key) + 3*hash(self._value)

    def unpack(self):
        return (self._key.unpack(), self._value.unpack())
