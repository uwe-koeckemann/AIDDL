from aiddl_core.function.function import LazyFunction
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.int import Int
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.substitution import Substitution


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
        if len(x) == 3:
            from_term = self.evaluator(x.get(0))
            to_term = self.evaluator(x.get(1))
            matchConstraint = x.get(2)

            s = from_term.match(to_term)

            if s is None:
                return Boolean.create(False)

            return self.evaluator(matchConstraint.substitute(s))
        else:
            p = x[0]
            for matchCase in x[1]:
                s = matchCase[0].match(p)
                if s is not None:
                    return self.evaluator(matchCase[1].substitute(s))
            raise ValueError("Match error:", x)


class EvalReferenceFunction:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        r = self.evaluator(x)
        return r


class EvalAllReferencesFunction:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        self.evaluator.set_follow_references(True)
        r = self.evaluator(x)
        self.evaluator.set_follow_references(False)
        return r