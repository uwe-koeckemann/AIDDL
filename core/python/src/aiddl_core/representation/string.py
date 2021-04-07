import aiddl_core.representation.term as term


class String(term.Term):
    __slots__ = ["_value"]

    def __init__(self, value):
        super(term.Term, self).__setattr__("_value", value)

    def get_string_value(self):
        return self._value.replace('\\"', '"')

    def resolve(self, container):
        return self

    def __str__(self):
        return '"%s"' % (str(self._value))

    def __repr__(self):
        return str(self._value)

    def __eq__(self, other):
        if isinstance(other, String):
            return self._value == other._value
        return False

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return 17*hash(self._value)

    def unpack(self):
        return self._value
