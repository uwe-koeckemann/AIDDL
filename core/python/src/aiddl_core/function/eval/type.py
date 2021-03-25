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
from aiddl_core.representation.function_reference import FunctionReference
from aiddl_core.representation.symbolic import TRUE
from aiddl_core.representation.symbolic import FALSE

import aiddl_core.function.uri as furi

SELF = Symbolic("#self")
SELF_ALT = Symbolic("#arg")

class EvalType:
    def __init__(self, freg):
        self.evaluator = freg.get_function(furi.EVAL)
        self.freg = freg

    def apply(self, args):
        x = args[0]
        type_def = args[1]
        if isinstance(type_def, Collection):
            for t in type_def:
                if t == FALSE:
                    return t
                answer = self.check_type(t, x)
                if answer == TRUE:
                    return answer
            return FALSE

        if type_def == FALSE:
            return FALSE
        return self.check_type(type_def, x)

    def check_type(self, type_def, x):
        tCheck = None
        if isinstance(type_def, Symbolic):
            tCheck = self.fReg.get_function(type_def)
            print("[W] Symbolic function reference:", type_def, "for", x, "it's a", type(type_def))
        elif isinstance(type_def, FunctionReference):
            tCheck = type_def.get_function()
            if tCheck is not None:
                return tCheck.apply(x)
            else:
                print("Function not found:", type_def)
        print("[W] Not a function reference:", type_def, "for", x, "it's a", type(type_def))
        selfSub = Substitution()
        selfSub.add(SELF, x)
        selfSub.add(SELF_ALT, x)
        return self.evaluator.apply(type_def.substitute(selfSub))


class IsTerm:
    def apply(self, x):
        return Boolean.create(True)


class IsSymbolic:
    def apply(self, x):
        return Boolean.create(isinstance(x, Symbolic))


class IsBoolean:
    def apply(self, x):
        return Boolean.create(isinstance(x, Symbolic))


class IsString:
    def apply(self, x):
        return Boolean.create(isinstance(x, String))


class IsNumerical:
    def apply(self, x):
        return Boolean.create(isinstance(x, Numerical))


class IsInteger:
    def apply(self, x):
        return Boolean.create(isinstance(x, Integer))


class IsRational:
    def apply(self, x):
        return Boolean.create(isinstance(x, Rational))


class IsReal:
    def apply(self, x):
        return Boolean.create(isinstance(x, Real))


class IsInfinity:
    def apply(self, x):
        return Boolean.create(isinstance(x, Real))


class IsCollection:
    def apply(self, x):
        return Boolean.create(isinstance(x, Collection))


class IsList:
    def apply(self, x):
        return Boolean.create(isinstance(x, List))


class IsTuple:
    def apply(self, x):
        return Boolean.create(isinstance(x, Tuple))


class IsSet:
    def apply(self, x):
        return Boolean.create(isinstance(x, Set))


class IsVariable:
    def apply(self, x):
        return Boolean.create(isinstance(x, Variable))


class IsKeyValue:
    def apply(self, x):
        return Boolean.create(isinstance(x, KeyValue))


class IsReference:
    def apply(self, x):
        return Boolean.create(isinstance(x, Reference))


class IsFunctionReference:
    def apply(self, x):
        return Boolean.create(isinstance(x, FunctionReference))
