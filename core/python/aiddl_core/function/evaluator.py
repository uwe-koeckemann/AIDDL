from aiddl_core.function.function import LazyFunction
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.var import Var
from aiddl_core.representation.num import Num
from aiddl_core.representation.str import Str
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.key_value import KeyValue

import aiddl_core.function.uri as furi

SELF = Sym("#self")
SELF_ALT = Sym("#arg")


class Evaluator:
    def __init__(self, freg, db):
        self.log_indent = ""
        self.verbose = False

        self.follow_references = False
        self.db = db
        self.selfStack = []
        self.cache = {}

        self.delayedEval = set()

        self.delayedEval.add(furi.LAMBDA)
        self.delayedEval.add(furi.AND)
        self.delayedEval.add(furi.OR)
        self.delayedEval.add(furi.FORALL)
        self.delayedEval.add(furi.EXISTS)
        self.delayedEval.add(furi.IF)
        self.delayedEval.add(furi.COND)

        self.delayedEval.add(furi.ZIP)
        self.delayedEval.add(furi.MATCH)
        self.delayedEval.add(furi.QUOTE)
        self.delayedEval.add(furi.DOMAIN)
        self.delayedEval.add(furi.LET)
        self.delayedEval.add(furi.MAP)
        self.delayedEval.add(furi.FILTER)
        self.freg = freg

    def set_follow_references(self, flag):
        self.follow_references = flag

    def set_verbose(self, flag):
        self.verbose = flag

    def set_container(self, db):
        self.db = db

    def evaluatable(self, arg):
        # answer = self.evaluatable_cache.get(arg)
        # if answer is not None:
        #     return answer
        answer = None
        if isinstance(arg, Sym):
            answer = False
        elif isinstance(arg, Num):
            answer = False
        elif isinstance(arg, Var):
            answer = False
        elif isinstance(arg, Str):
            answer = False
        elif isinstance(arg, EntRef):
            answer = True
        elif isinstance(arg, FunRef):
            answer = False
        elif isinstance(arg,  KeyValue):
            answer = self.evaluatable(arg.get_key()) \
                     or self.evaluatable(arg.get_value())
        elif isinstance(arg, Collection):
            for c in arg:
                if self.evaluatable(c):
                    answer = True
                    break
            if answer is None:
                answer = False
        elif isinstance(arg, Tuple):
            if len(arg) == 0:
                answer = False
            elif self.freg.has_function(arg.get(0)):
                answer = True
            if answer is None:
                for e in arg:
                    if self.evaluatable(e):
                        answer = True
                        break
        if answer is None:
            answer = False

        # self.evaluatable_cache[arg] = answer
        return answer

    def __call__(self, arg):
        # if not self.evaluatable(arg):
        #     return arg
        # if arg in self.cache.keys():
        #     return self.cache[arg]

        if self.verbose:
            print(self.log_indent + str(arg))
            self.log_indent += "  "

        operator = None

        if not isinstance(arg, Tuple) or len(arg) == 0:
            if isinstance(arg, Collection):
                new_col = []
                for t in arg:
                    new_col.append(self(t))

                if isinstance(arg, List):
                    result = List(new_col)
                else:
                    result = Set(new_col)
            elif isinstance(arg, KeyValue):
                result = KeyValue(
                    self(arg.get_key()), self(arg.get_value()))
            elif isinstance(arg, EntRef):
                if self.follow_references:
                    result = self(self.db.resolve_reference(arg))
                else:
                    result = self.db.resolve_reference(arg)
            else:
                result = arg
        else:
            operator = arg.get(0)
            if isinstance(operator, EntRef):
                target = operator.get_ref_target()
                if isinstance(target, Sym):
                    uri = operator.convert2uri()
                    if self.freg.has_function(uri):
                        operator = uri
                    else:
                        operator = self.db.resolve_reference(operator)
                else:
                    operator = self.db.resolve_reference(operator)

            resolved_arguments = []
            processed_op = self(operator)
            is_lazy = False
            if not self.freg.has_function(processed_op):
                resolved_arguments.append(processed_op)
            else:
                is_lazy = isinstance(self.freg.get_function(operator), LazyFunction)
            for i in range(1, arg.size()):
                if not is_lazy:
                    if isinstance(arg.get(i), EntRef):
                        res_arg = self.db.resolve_reference(arg.get(i))
                    else:
                        res_arg = arg.get(i)

                    if not is_lazy:
                        resolved_arguments.append(self(res_arg))
                    else:
                        resolved_arguments.append(res_arg)
                else:
                    resolved_arguments.append(arg.get(i))
            if len(resolved_arguments) == 1:
                processed_args = resolved_arguments[0]
            else:
                processed_args = Tuple(resolved_arguments)
            if self.freg.has_function(processed_op):
                result = self.freg.get_function(processed_op)(processed_args)
            else:
                result = processed_args
 
        if self.verbose:
            if operator is None:
                operator = Sym("-")
            self.log_indent = self.log_indent[0:-2]
            print(self.log_indent + str(result) + "//" + str(operator))

        self.cache[arg] = result
        return result

    # def register(fName, f, delayed):
    #     if fName not in self.fMap.keys():
    #         self.fMap[fName] = f
    #     if delayed:
    #         self.delayedEval.add(fName)
