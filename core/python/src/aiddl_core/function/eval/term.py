from aiddl_core.representation.term import Term
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.symbolic import Boolean
from aiddl_core.representation.string import String
from aiddl_core.representation.variable import Variable
from aiddl_core.representation.numerical import Numerical
from aiddl_core.representation.rational import Rational
from aiddl_core.representation.real import Real
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.reference import Reference
from aiddl_core.representation.key_value import KeyValue
from aiddl_core.representation.substitution import Substitution

from aiddl_core.function.uri import EVAL
from aiddl_core.function.uri import QUOTE
from aiddl_core.function.uri import TYPE


class AtIndex:
    def apply(self, args: Tuple) -> Term:
        return args[1][args[0].int_value()]


class AtKey:
    def apply(self, args: Tuple) -> Term:
        return args[1][args[0]]


class First:
    def apply(self, args):
        return args[0]


class Last:
    def apply(self, args):
        return args[-1]


class GetKey:
    def apply(self, x):
        return x.get_key()


class GetValue:
    def apply(self, x):
        return x.get_value()


class ContainsKey:
    def apply(self, args: Tuple) -> Boolean:
        C = args[0]
        k = args[1]
        return Boolean.create(C.contains_key(k))


class Size:
    def apply(self, args):
        return Integer(len(args))


class SubstitutionFunction:
    def apply(self, x):
        s = Substitution.from_term(x[1])
        return x[0].substitute(s)


class Equals:
    def apply(self, x):
        return Boolean.create(x[0] == x[1])


class NotEquals:
    def apply(self, x):
        return Boolean.create(x[0] != x[1])


class Quote:
    def apply(self, x):
        return x


class EvalRef:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        return self.evaluator.apply(x)


class Matches:
    def apply(self, args: Tuple) -> Boolean:
        x = args.get(0)
        y = args.get(1)
        sub = x.match(y)
        return Boolean.create(sub is not None)


class CallFunction():
    def __init__(self, freg):
        self.freg = freg

    def apply(self, args):
        f = args[0]
        f_args = args[1]
        return f.get_function_or_panic().apply(f_args)


class ExpandDomain:
    MinKey = Symbolic("min")
    MaxKey = Symbolic("max")
    IncKey = Symbolic("inc")

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if not isinstance(x, Collection):
            return x
        return Set(self.evalDomain(x))

    def evalDomain(self, C):
        D = set()

        if C.contains_key(self.MinKey):
            min = self.evaluator.apply(C.get(self.MinKey)).int_value()
            max = self.evaluator.apply(C.get(self.MaxKey)).int_value()
            inc = self.evaluator.apply(C.get_or_default(self.IncKey,
                                                          Integer(1))).int_value()
            for i in range(min, max+1, inc):
                D.add(Integer(i))
        else:
            for e in C:
                e = self.evaluator.apply(e)
                if isinstance(e, Collection):
                    D.addAll(self.evalDomain(e))
                else:
                    D.add(e)
        return D


class Let:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        s = Substitution()
        for kvp in x.get(0):
            s.add(kvp.get_key(), self.evaluator.apply(kvp.get_value()))
        return self.evaluator.apply(x.get(1).substitute(s))


class Match:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        from_term = self.evaluator.apply(x.get(0))
        to_term = self.evaluator.apply(x.get(1))
        matchConstraint = x.get(2)

        s = from_term.match(to_term)

        if s is None:
            return Boolean.create(False)

        return self.evaluator.apply(matchConstraint.substitute(s))


class Signature:
    TYPE = Symbolic("org.aiddl.eval.type")
    QUOTE = Symbolic("org.aiddl.eval.quote")

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        targets = x.get(0)
        types = x.get(1)
        if not isinstance(targets, Tuple):
            return Boolean.create(True)

        for i in range(targets.size()):
            idx = min(i, types.size()-1)
            con = Tuple([TYPE, Tuple([QUOTE, targets.get(i)]), types.get(idx)])
            if not self.evaluator.apply(con).bool_value():
                return Boolean.create(False)
        return Boolean.create(True)


class Zip:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        matchTuple = x.get(0)
        zipTerm = x.get(1)
        sub_f = x.get(2)

        zipTerm = self.evaluator.apply(zipTerm)

        zipList = []
        for t in zipTerm:
            zipList.append(t)

        for i in range(len(zipList)-1):
            if len(zipList[i]) != len(zipList[i+1]):
                return Boolean.create(False)

        for i in range(len(zipList[0])):
            zipArgs = [None] * len(zipList)
            for j in range(len(zipList)):
                zipArgs[j] = zipList[j].get(i)

            zipTuple = Tuple(zipArgs)
            s = matchTuple.match(zipTuple)
            if not self.evaluator.apply(sub_f.substitute(s)).bool_value():
                return Boolean.create(False)

        return Boolean.create(True)


class Map:
    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def apply(self, args):
        f = self.evaluator.apply(args[0]).get_function_or_panic()        
        C = self.evaluator.apply(args[1])

        C_m = []
        for e in C:
            C_m.append(f.apply(e))

        if isinstance(C, List):
            return List(C_m)
        else:
            return Set(C_m)


class Filter:
    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def apply(self, args):
        f = self.evaluator.apply(args[0]).get_function_or_panic()
        C = self.evaluator.apply(args[1])
        C_m = []
        for e in C:
            if f.apply(e).bool_value():
                C_m.append(e)

        if isinstance(C, List):
            return List(C_m)
        else:
            return Set(C_m)


class Reduce:
    INIT_VAL = Symbolic("initial-value")

    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(EVAL)

    def apply(self, args):
        f = self.evaluator.apply(args[0]).get_function_or_panic()        
        C = self.evaluator.apply(args[1])
        current_value = args.get(Reduce.INIT_VAL)

        for e in C:
            if current_value is None:
                current_value = e
            else:
                current_value = f.apply(Tuple([current_value, e]))
        return current_value
