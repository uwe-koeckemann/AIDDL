from aiddl_core.representation.sym import Sym
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.sym import TRUE
from aiddl_core.representation.sym import FALSE

import aiddl_core.function.uri as furi

SELF = Sym("#self")
SELF_ALT = Sym("#arg")


class EvalType:
    def __init__(self, freg):
        self.evaluator = freg.get_function(furi.EVAL)
        self.freg = freg

    def __call__(self, args):
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
        t_check = None
        if isinstance(type_def, Sym):
            t_check = self.fReg.get_function(type_def)
            print("[W] Symbolic function reference:", type_def, "for", x)
        elif isinstance(type_def, FunRef):
            t_check = type_def.get_function()
            if t_check is not None:
                return t_check(x)
            else:
                print("Function not found:", type_def)
        print("[W] Not a function reference:", type_def, "for:", x, "it has type:", type(type_def))
        self_sub = Substitution()
        self_sub.add(SELF, x)
        self_sub.add(SELF_ALT, x)
        return self.evaluator(type_def.substitute(self_sub))

