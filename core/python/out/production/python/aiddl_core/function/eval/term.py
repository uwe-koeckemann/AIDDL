from aiddl_core.function.function import LazyFunction
from aiddl_core.representation.term import Term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.int import Int
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.substitution import Substitution

from aiddl_core.function.uri import EVAL
from aiddl_core.function.uri import QUOTE
from aiddl_core.function.uri import TYPE


class SubstitutionFunction:
    def __call__(self, x):
        s = Substitution.from_term(x[1])
        return x[0].substitute(s)


class Quote(LazyFunction):
    def __call__(self, x):
        return x


class EvalRef:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        return self.evaluator(x)


class Matches:
    def __call__(self, args: Tuple) -> Boolean:
        x = args.get(0)
        y = args.get(1)
        sub = x.match(y)
        return Boolean.create(sub is not None)


class CallFunction:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, args):
        f = args[0]
        f_args = args[1]
        return f.get_function_or_panic()(f_args)


class ExpandDomain(LazyFunction):
    MinKey = Sym("min")
    MaxKey = Sym("max")
    IncKey = Sym("inc")

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        if not isinstance(x, Collection):
            return x
        return Set(self.evalDomain(x))

    def evalDomain(self, C):
        D = set()

        if C.contains_key(self.MinKey):
            min = self.evaluator(C.get(self.MinKey)).int_value()
            max = self.evaluator(C.get(self.MaxKey)).int_value()
            inc = self.evaluator(C.get_or_default(self.IncKey,
                                                  Int(1))).int_value()
            for i in range(min, max+1, inc):
                D.add(Int(i))
        else:
            for e in C:
                e = self.evaluator(e)
                if isinstance(e, Collection):
                    D.addAll(self.evalDomain(e))
                else:
                    D.add(e)
        return D


class Let(LazyFunction):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        s = Substitution()
        for kvp in x.get(0):
            s.add(kvp.get_key(), self.evaluator(kvp.get_value()))
        return self.evaluator(x.get(1).substitute(s))


class Match(LazyFunction):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        from_term = self.evaluator(x.get(0))
        to_term = self.evaluator(x.get(1))
        matchConstraint = x.get(2)

        s = from_term.match(to_term)

        if s is None:
            return Boolean.create(False)

        return self.evaluator(matchConstraint.substitute(s))


class Signature:
    TYPE = Sym("org.aiddl.eval.type")
    QUOTE = Sym("org.aiddl.eval.quote")

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        targets = x.get(0)
        types = x.get(1)
        if not isinstance(targets, Tuple):
            return Boolean.create(True)

        for i in range(targets.size()):
            idx = min(i, types.size()-1)
            con = Tuple([TYPE, Tuple([QUOTE, targets.get(i)]), types.get(idx)])
            if not self.evaluator(con).bool_value():
                return Boolean.create(False)
        return Boolean.create(True)





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
