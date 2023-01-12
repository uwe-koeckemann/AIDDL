from aiddl_core.function.function import FunctionMixin, LazyFunctionMixin
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.int import Int
from aiddl_core.representation.num import Num
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set


class ContainsMatch(FunctionMixin):
    def __call__(self, args: Tuple) -> Boolean:
        c = args[0]
        e = args[1]
        for e_c in c:
            if e.match(e_c) is not None:
                return Boolean(True)
        return Boolean(False)


class ContainsAny(FunctionMixin):
    def __call__(self, args: Tuple) -> Boolean:
        c1 = args[0]
        c2 = args[1]
        for e in c2:
            if e in c1:
                return Boolean(True)
        return Boolean(False)


class Sum(FunctionMixin):
    def __call__(self, c: Collection) -> Num:
        s = Int(0)
        for e in c:
            s += e
        return s


class Union(FunctionMixin):
    def __call__(self, x: Collection) -> Set:
        u = set()
        for s in x:
            for e in s:
                u.add(e)
        return Set(u)


class Concat(FunctionMixin):
    def __call__(self, x):
        cl = []
        for s in x:
            for e in s:
                cl.append(e)
        return List(cl)


class Zip(FunctionMixin, LazyFunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        zip_term = self.evaluator(x)

        zip_list = []
        for t in zip_term:
            if isinstance(t, Tuple) or isinstance(t, List):
                zip_list.append(t)
            else:
                raise ValueError("Bad argument for zip: %s. Can only zip tuples or lists." % (str(t)))

        zipped_list = []
        idx = 0
        done = False
        while not done:
            zip_line = []
            for i in range(0, len(zip_list)):
                if idx >= len(zip_list[i]):
                    done = True
                    break
                zip_line.append(zip_list[i][idx])
            if not done:
                zipped_list.append(Tuple(zip_line))
                idx += 1
        return List(zipped_list)