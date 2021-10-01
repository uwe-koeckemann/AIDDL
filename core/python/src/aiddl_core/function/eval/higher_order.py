from aiddl_core.function.function import LazyFunction
from aiddl_core.function.uri import EVAL
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.tuple import Tuple


class Map(LazyFunction):
    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def __call__(self, args):
        f = self.evaluator(args[0]).get_function_or_panic()
        C = self.evaluator(args[1])

        C_m = []
        for e in C:
            C_m.append(f(e))

        if isinstance(C, List):
            return List(C_m)
        else:
            return Set(C_m)


class Filter(LazyFunction):
    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def __call__(self, args):
        f = self.evaluator(args[0]).get_function_or_panic()
        C = self.evaluator(args[1])
        C_m = []
        for e in C:
            if f(e).bool_value():
                C_m.append(e)

        if isinstance(C, List):
            return List(C_m)
        else:
            return Set(C_m)


class Reduce(LazyFunction):
    INIT_VAL = Sym("initial-value")

    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def __call__(self, args):
        f = self.evaluator(args[0]).get_function_or_panic()
        C = self.evaluator(args[1])
        current_value = args.get(Reduce.INIT_VAL)

        for e in C:
            if current_value is None:
                current_value = e
            else:
                current_value = f(Tuple([current_value, e]))
        return current_value


class CallFunction:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, args):
        f = args[0]
        f_args = args[1]
        return f.get_function_or_panic()(f_args)