from aiddl_core.representation.str import Str


class StringConcatFunction:
    def __call__(self, x):
        s = '"'
        for e in x:
            s += e.string
        s += '"'
        return Str(s)
