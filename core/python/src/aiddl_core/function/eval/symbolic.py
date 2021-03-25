from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.list import List


class Concat:
    def apply(self, x):
        return x[0] + x[1]


class Split:
    def apply(self, x):
        split_list = []
        for t in str(x).split("."):
            split_list.append(Symbolic(t))
        return List(split_list)
