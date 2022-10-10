from aiddl_core.function.function import FunctionMixin, LazyFunctionMixin
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.int import Int
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.substitution import Substitution


class SubstitutionFunction(FunctionMixin):
    def __call__(self, x):
        s = Substitution.from_term(x[1])
        return x[0].substitute(s)


class Quote(FunctionMixin, LazyFunctionMixin):
    def __call__(self, x):
        return x


class EvalRef(FunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        return self.evaluator(x)


class Matches(FunctionMixin):
    def __call__(self, args: Tuple) -> Boolean:
        x = args.get(0)
        y = args.get(1)
        sub = x.match(y)
        return Boolean.create(sub is not None)


class ExpandDomain(FunctionMixin, LazyFunctionMixin):
    MinKey = Sym("min")
    MaxKey = Sym("max")
    IncKey = Sym("inc")

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        if not isinstance(x, Collection):
            return x
        return Set(self._eval_domain(x))

    def _eval_domain(self, C):
        domain = set()

        if C.contains_key(self.MinKey):
            min_value = self.evaluator(C.get(self.MinKey)).int_value
            max_value = self.evaluator(C.get(self.MaxKey)).int_value
            inc = self.evaluator(C.get_or_default(self.IncKey,
                                                  Int(1))).int_value
            for i in range(min_value, max_value+1, inc):
                domain.add(Int(i))
        else:
            for e in C:
                e = self.evaluator(e)
                if isinstance(e, Collection):
                    domain.addAll(self._eval_domain(e))
                else:
                    domain.add(e)
        return domain


class Let(FunctionMixin, LazyFunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        s = Substitution()
        for kvp in x.get(0):
            s.add(kvp.key, self.evaluator(kvp.value))
        return self.evaluator(x.get(1).substitute(s))


class Match(FunctionMixin, LazyFunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        if len(x) == 3:
            from_term = self.evaluator(x.get(0))
            to_term = self.evaluator(x.get(1))
            match_constraint = x.get(2)

            s = from_term.match(to_term)

            if s is None:
                return Boolean.create(False)

            return self.evaluator(match_constraint.substitute(s))
        else:
            p = x[0]
            for matchCase in x[1]:
                s = matchCase[0].match(p)
                if s is not None:
                    return self.evaluator(matchCase[1].substitute(s))
            raise ValueError("Match error:", x)


class EvalReferenceFunction(FunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        r = self.evaluator(x)
        return r


class EvalAllReferencesFunction(FunctionMixin):
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def __call__(self, x):
        self.evaluator.set_follow_references(True)
        r = self.evaluator(x)
        self.evaluator.set_follow_references(False)
        return r