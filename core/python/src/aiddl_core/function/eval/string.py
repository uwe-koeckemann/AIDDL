from aiddl_core.representation.string import String


class StringConcatFunction:
    def apply(self, x):
        s = '"'
        for e in x:
            s += e.get_string_value()
        s += '"'
        return String(s)
