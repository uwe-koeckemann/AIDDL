from aiddl_core.representation.sym import Sym
from aiddl_core.representation.list import List


class Split:
    def __call__(self, x):
        split_list = []
        for t in str(x).split("."):
            split_list.append(Sym(t))
        return List(split_list)
