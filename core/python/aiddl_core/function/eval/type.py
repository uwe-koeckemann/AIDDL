from aiddl_core.representation.inf import Inf
from aiddl_core.representation.int import Int
from aiddl_core.representation.keyval import KeyVal
from aiddl_core.representation.list import List
from aiddl_core.representation.num import Num
from aiddl_core.representation.set import Set
from aiddl_core.representation.sym import Sym, Boolean
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.sym import TRUE
from aiddl_core.representation.sym import FALSE

import aiddl_core.function as furi
from aiddl_core.representation.tuple import Tuple
from aiddl_core.util.logger import Logger

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
        if isinstance(type_def, Sym):
            t_check = self.fReg.get_function(type_def)
            print("[W] Symbolic function reference:", type_def, "for", x)
            return t_check(x)
        elif isinstance(type_def, FunRef):
            if type_def.function is not None:
                return type_def.function(x)
            else:
                print("Function not found:", type_def)
        print("[W] Not a function reference:", type_def, "for:", x, "it has type:", type(type_def))
        self_sub = Substitution()
        self_sub.add(SELF, x)
        self_sub.add(SELF_ALT, x)
        return self.evaluator(type_def.substitute(self_sub))


class TypeCheckFunction:
    def __init__(self, type_def, evaluator):
        self.type_def = type_def
        self.evaluator = evaluator

    def __call__(self, term):
        return Boolean.create(self.check(self.type_def, term))

    def check(self, type_def, term):
        r = False
        if isinstance(type_def, Tuple):
            type_class = type_def[0]

            if type_class == Sym("basic-type"):
                e = Tuple([type_def[1], term])
                r = self.evaluator(e).bool
            elif type_class == Sym("org.aiddl.type.set-of"):
                sub_type = type_def.get(1)
                if isinstance(term, Set):
                    r = True
                    Logger.inc_depth()
                    for e in term:
                        if not self.check(sub_type, e):
                            #print("---> FAIL")
                            r = False
                            break
                    Logger.dec_depth()
                else:
                    r = False
            elif type_class == Sym("org.aiddl.type.list-of"):
                subType = type_def.get(1)
                if isinstance(term, List):
                    r = True
                    Logger.inc_depth()
                    for e in term:
                        if not self.check(subType, e):
                            r = False
                            break
                    Logger.dec_depth()
                else:
                    r = False
            elif type_class == Sym("org.aiddl.type.collection-of"):
                subType = type_def.get(1)
                if isinstance(term, Collection):
                    r = True
                    Logger.inc_depth()
                    for e in term:
                        if not self.check(subType, e):
                            r = False
                            break
                    Logger.dec_depth()
                else:
                    r = False
            elif type_class == Sym("org.aiddl.type.tuple.signed"):
                if isinstance(term, Tuple):
                    signature = type_def.get(1)
                    min = type_def.get_or_default(Sym("min"), Int(len(signature)))
                    max = type_def.get_or_default(Sym("max"), Int(len(signature)))
                    repeat = type_def.get_or_default(Sym("repeat"), Int(1))
                    tSize = Int(len(term))
                    repeat_start_idx = len(signature) - repeat.int_value

                    if tSize >= min and tSize <= max:
                        Logger.inc_depth()
                        r = True
                        for i in range(0, len(signature)):
                            sig_idx = i
                            if i >= signature.size():
                                sig_idx = repeat_start_idx + \
                                    (i - signature.size()) % repeat.int_value
                                if not self.check(signature.get(sig_idx), term.get(i)):
                                    r = False
                                    break
                        Logger.dec_depth()
                    else:
                        r = False
                else:
                    r = False
            elif type_class == Sym("org.aiddl.type.dictionary"):
                keyTypeCol = type_def.get(1)
                r = True
                Logger.inc_depth()
                for kvp in keyTypeCol:
                    e = term.get(kvp.key)
                    if e is None:
                        r = False
                        break
                    if not self.check(kvp.value, e):
                        r = False
                        break

                optional = type_def.get_or_default(Sym("optional"), Set([]))
                for kvp in optional:
                    e = term.get(kvp.key)
                    if e is not None:
                        if not self.check(kvp.value, e):
                            r = False
                            break

                Logger.dec_depth()
            elif type_class == Sym("org.aiddl.type.matrix"):
                if (isinstance(term, Tuple) or isinstance(term, List)) and len(term) > 0:
                    m = type_def.get_or_default(Sym("m"), Int(len(term))).unpack()
                    n = type_def.get_or_default(Sym("n"), Int(len(term[0]))).unpack()
                    cell_type = type_def.get(Sym("cell-type"))
                    row_types = type_def.get(Sym("row-type"))
                    col_types = type_def.get(Sym("col-type"))
                    r = True
                    if len(term) != m:
                        r = False
                    else:
                        for i in range(m):
                            if not r or len(term[i]) != n:
                                r = False
                                break
                            for j in range(m):
                                cell_okay = cell_type is not None and self.check(cell_type, term[i][j])
                                row_okay = row_types is not None and self.check(row_types[i], term[i][j])
                                col_okay = col_types is not None and self.check(col_types[j], term[i][j])
                                if not (cell_okay and row_okay and col_okay):
                                    r = False
                                    break
                else:
                    r = False

            elif type_class == Sym("org.aiddl.type.enum"):
                r = term in type_def.get(1)
            elif type_class == Sym("org.aiddl.type.range"):
                min_val = type_def.get_or_default(Sym("min"), Inf.neg())
                max_val = type_def.get_or_default(Sym("max"), Inf.pos())
                r = False
                if isinstance(term, Num):
                    if min_val <= term <= max_val:
                        r = True
            elif type_class == Sym("org.aiddl.type.typed-key-value"):
                r = False
                if isinstance(term, KeyVal):
                    if self.check(type_def[1].key, term.key) and self.check(type_def[1].value, term.value):
                        r = True
            elif type_class == Sym("org.aiddl.type.union"):
                r = False
                for choice in type_def[1]:
                    if self.check(choice, term):
                        r = True
                        break
            elif type_class == Sym("org.aiddl.type.intersection"):
                r = True
                for choice in type_def[1]:
                    if not self.check(choice, term):
                        r = False
                        break
            else:
                raise ValueError("#type expression not supported:", type_def)
        elif isinstance(type_def, Sym):
            e = Tuple([type_def, term])
            r = self.evaluator(e).bool
        elif isinstance(type_def, FunRef):
            r = type_def(term).bool
        else:
            r = False
            raise ValueError("#type expression not supported (%s): %s" % (str(type(type_def)), str(type_def)))

        if r and isinstance(type_def, Tuple):
            constraint = type_def[Sym("constraint")]
            if constraint is not None and isinstance(constraint, FunRef):
                r = constraint.function(term).bool
                # if not r:
                #     if eval.get_verbosity() >= 1:
                #         Logger.incDepth()
                #         Logger.msg("TypeCheck", t, " does not satisfy " + constraint)
                #         Logger.decDepth()

        # if not r:
        #     Logger.msg("TypeCheck", str(term) + " !!  " + str(type_def))
        return r


class GenericTypeConstructor:
    def __init__(self, uri_base, args, type_def, evaluator, freg):
        self.next_free_id = 0
        self.uri_base = uri_base
        self.args = args
        self.type_def = type_def
        self.evaluator = evaluator
        self.freg = freg

    def __call__(self, x):
        self.next_free_id += 1
        s = self.args.match(x)
        type_def = self.type_def.substitute(s)
        uri = self.uri_base + Sym("n%d" % self.next_free_id)
        print(type_def)
        type_fun = TypeCheckFunction(type_def, self.evaluator)
        self.freg.add_function(uri, type_fun)
        return FunRef(uri, self.freg)
