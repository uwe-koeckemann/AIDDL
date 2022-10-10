import aiddl_core.representation.num as numerical


class NaN(numerical.Num):
    @staticmethod
    def nan():
        return NaN()

    def __init__(self):
        pass

    def resolve(self, container):
        return self

    def __add__(self, other):
        return self

    def __sub__(self, other):
        return self

    def __mul__(self, other):
        return self

    def __truediv__(self, other):
        return self

    def __floordiv__(self, other):
        return self

    def __str__(self):
        return "NaN"

    def __eq__(self, other):
        return False

    def __ne__(self, other):
        return True

    def __lt__(self, other):
        return False

    def __le__(self, other):
        return False

    def __gt__(self, other):
        return False

    def __ge__(self, other):
        return False

    def is_zero(self):
        return False

    def is_positive(self):
        return False

    def is_negative(self):
        return False

    def is_nan(self):
        return True

    def __hash__(self):
        return 17*hash('NaN')

    def unpack(self):
        return float('NaN')
