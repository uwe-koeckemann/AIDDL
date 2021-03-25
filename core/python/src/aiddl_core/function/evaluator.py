from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.symbolic import Boolean
from aiddl_core.representation.variable import Variable
from aiddl_core.representation.numerical import Numerical
from aiddl_core.representation.rational import Rational
from aiddl_core.representation.real import Real
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.string import String
from aiddl_core.representation.function_reference import FunctionReference
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.reference import Reference
from aiddl_core.representation.key_value import KeyValue
from aiddl_core.representation.substitution import Substitution

import aiddl_core.function.uri as furi

SELF = Symbolic("#self")
SELF_ALT = Symbolic("#arg")

class Evaluator:
    def __init__(self, freg, db):
        self.log_indent = ""
        self.verbose = False

        self.follow_references = False
        self.db = db
        self.selfStack = []
        self.cache = {}
        self.evaluatable_cache = {}

        self.delayedEval = set()

        self.delayedEval.add(furi.LAMBDA)
        self.delayedEval.add(furi.FORALL)
        self.delayedEval.add(furi.EXISTS)
        self.delayedEval.add(furi.MATCH)
        self.delayedEval.add(furi.IF)
        self.delayedEval.add(furi.COND)
        self.delayedEval.add(furi.ZIP)
        self.delayedEval.add(furi.AND)
        self.delayedEval.add(furi.OR)
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
        if isinstance(arg, Symbolic):
            answer = False
        elif isinstance(arg, Numerical):
            answer = False
        elif isinstance(arg, Variable):
            answer = False
        elif isinstance(arg, String):
            answer = False
        elif isinstance(arg, Reference):
            answer = True
        elif isinstance(arg, FunctionReference):
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

    def apply(self, arg):
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
                newCol = []
                for t in arg:
                    newCol.append(self.apply(t))

                if isinstance(arg, List):
                    result = List(newCol)
                else:
                    result = Set(newCol)
            elif isinstance(arg, KeyValue):
                result = KeyValue(
                    self.apply(arg.get_key()), self.apply(arg.get_value()))
            elif isinstance(arg, Reference):
                #print("-----", arg, self.follow_references)
                if self.follow_references:
                    result = self.apply(self.db.resolve_reference(arg))
                else:
                    result = self.db.resolve_reference(arg)
                #print(arg, "resolved to", result)
            else:
                result = arg
        else:
            operator = arg.get(0)
            if isinstance(operator, Reference):
                target = operator.get_ref_target()
                if isinstance(target, Symbolic):
                    uri = operator.convert2uri()
                    if self.freg.has_function(uri):
                        operator = uri
                    else:
                        operator = self.db.resolve_reference(operator)
                else:
                    operator = self.db.resolve_reference(operator)

                    # if self.verbose:
            #     tail = str(operator)

            resolvedArguments = []
            processed_op = self.apply(operator)
            if not self.freg.has_function(processed_op):
                resolvedArguments.append(processed_op)
            for i in range(1, arg.size()):
                if operator not in self.delayedEval:
                    if isinstance(arg.get(i), Reference):
                        res_arg = self.db.resolve_reference(arg.get(i))
                    else:
                        res_arg = arg.get(i)

                    # if len(self.selfStack) > 0:
                    #     if operator != TYPE:
                    #         res_arg = res_arg.substitute(self.selfStack[-1])
                    #     elif i == 1:
                    #         res_arg = res_arg.substitute(self.selfStack[-1])

                    # if operator == TYPE and i == 2:
                    #     s = Substitution()
                    #     s.add(SELF, resolvedArguments[1])
                    #     res_arg = res_arg.substitute(s)
                    #     self.selfStack.append(s)
                    #     doPop = True

                    if operator not in self.delayedEval:
                        resolvedArguments.append(self.apply(res_arg))
                    else:
                        resolvedArguments.append(res_arg)
                else:
                    resolvedArguments.append(arg.get(i))
            if len(resolvedArguments) == 1:
                processedArgs = resolvedArguments[0]
            else:
                processedArgs = Tuple(resolvedArguments)
            if self.freg.has_function(operator):
                result = self.freg.get_function(operator).apply(processedArgs)
            else:
                result = processedArgs
 
        if self.verbose:
            if operator is None:
                operator = Symbolic("-")
            self.log_indent = self.log_indent[0:-2]
            print(self.log_indent + str(result) + "//" + str(operator))

        self.cache[arg] = result
        return result

    # def register(fName, f, delayed):
    #     if fName not in self.fMap.keys():
    #         self.fMap[fName] = f
    #     if delayed:
    #         self.delayedEval.add(fName)
