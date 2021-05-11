import importlib

from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.symbolic import Boolean
from aiddl_core.representation.numerical import Numerical
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.infinity import Infinity
from aiddl_core.representation.key_value import KeyValue
from aiddl_core.representation.list import List
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.set import Set
from aiddl_core.representation.function_reference import FunctionReference

from aiddl_core.tools.logger import Logger

import aiddl_core.function.uri as furi

SELF = Symbolic("#self")
SELF_ALT = Symbolic("#arg")


class NamedFunction:
    def __init__(self, name, f, e, args=None):
        self.name = name
        self.f = f
        self.e = e
        self.args = args

    def apply(self, arg):
        if self.args is None:
            s = Substitution()
            s.add(SELF, arg)
            s.add(SELF_ALT, arg)
        else:
            s = self.args.match(arg)
        return self.e.apply(self.f.substitute(s))


class PythonFunctionLoader:
    def __init__(self, freg):
        self.freg = freg

    def apply(self, x):
        name = x[Symbolic("name")]
        module = x[Symbolic("module")]
        py_class = x[Symbolic("pyclass")]
        py_module = x[Symbolic("pymodule")]

        imp = importlib.import_module(str(py_module))
        instance = getattr(imp, str(py_class))()
        uri = module + name

        self.freg.add_function(uri, instance)


class PythonFunctionFactoryLoader:
    def __init__(self, freg):
        self.freg = freg

    def apply(self, x):
        name = x[Symbolic("name")]
        module = x[Symbolic("module")]
        py_class = x[Symbolic("pyclass")]
        py_module = x[Symbolic("pymodule")]

        imp = importlib.import_module(str(py_module))
        constructor = getattr(imp, str(py_class))

        factory = PythonFunctionFactory(constructor, self.freg)
        uri = module + name

        self.freg.add_function(uri, factory)
        return FunctionReference(uri, self.freg)


class PythonFunctionFactory:
    def __init__(self, constructor, freg):
        self.constructor = constructor
        self.freg = freg

    def apply(self, x):
        uri = None
        init = None
        config = None
        if isinstance(x, Symbolic):
            uri = x
        else:
            uri = x[0]
            init = x[Symbolic("init")]
            config = x[Symbolic("config")]

        f = self.constructor()

        if init is not None:
            f.initialize(init)

        if config is not None:
            f.configure(config, self.freg)

        self.freg.add_function(uri, f)
        return FunctionReference(uri, self.freg)


class CoreLang:
    Lang = Symbolic("python")

    def apply(self, x):
        return CoreLang.Lang


class EvalReferenceFunction:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        r = self.evaluator.apply(x)
        return r


class EvalAllReferencesFunction:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        self.evaluator.set_follow_references(True)
        r = self.evaluator.apply(x)
        self.evaluator.set_follow_references(False)
        return r


class CallRequestFunction:
    def __init__(self, C, req_handler):
        self.C = C
        self.req_handler = req_handler

    def apply(self, x):
        request = x[0]
        module = x[1]
        return_entry_name = x[2]

        self.C.add_module(module)
        # self.req_handler.verbose = True
        # print(request)
        self.req_handler.satisfy_request(request, module)

        r = self.C.get_entry(return_entry_name, module=module).get_value()
        return r


class InitFunction:
    def __init__(self, freg):
        self.freg = freg

    def apply(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Symbolic):
            func = self.freg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunctionReference):
            func = f.getFunction()
            uri = f.getFunRefTerm()

        func.initialize(args)
        return FunctionReference(uri, self.freg)


class ConfigFunction:
    def __init__(self, freg):
        self.freg = freg

    def apply(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Symbolic):
            func = self.freg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunctionReference):
            func = f.getFunction()
            uri = f.getFunRefTerm()

        func.configure(args, self.freg)
        return FunctionReference(uri, self.freg)


class LambdaEvaluator:
    NextID = 0

    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(furi.EVAL)

    def apply(self, arg):
        arg_term = arg[0]
        fun_term = arg[1]

        f = LambdaFunction(arg_term, fun_term, self.evaluator)
        uri = Symbolic("#lambda_%d" % (LambdaEvaluator.NextID))
        LambdaEvaluator.NextID += 1
        self.freg.add_function(uri, f)
        return FunctionReference(uri, self.freg)


class LambdaFunction:
    def __init__(self, x, f, e):
        self.x = x
        self.f = f
        self.e = e

    def apply(self, arg):
        s = self.x.match(arg)
        return self.e.apply(self.f.substitute(s))


class GetMatchingEntries:
    def __init__(self, C):
        self.C = C

    def apply(self, x):
        module_pattern = x[0]
        type_pattern = x[1]
        name_pattern = x[2]

        result = self.C.get_matching_entries(module_pattern,
                                             type_pattern,
                                             name_pattern)
        L = []
        for e in result:
            L.append(e.as_tuple())

        return List(L)


class TypeCheckFunction:
    def __init__(self, type_def, evaluator):
        self.type_def = type_def
        self.evaluator = evaluator

    def apply(self, term):
        return Boolean.create(self.check(self.type_def, term))

    def check(self, type_def, term):
        # Logger.msg("TypeCheck", str(type_def) + " ??  " + str(term))
        r = False
        if isinstance(type_def, Tuple):
            type_class = type_def[0]

            if type_class == Symbolic("basic-type"):
                e = Tuple([type_def[1], term]) 
                r = self.evaluator.apply(e).bool_value()
            elif type_class == Symbolic("set-of"):
                subType = type_def.get(1)
                if isinstance(term, Set):
                    r = True
                    Logger.inc_depth()
                    for e in term:
                        if not self.check(subType, e):
                            #print("---> FAIL")
                            r = False
                            break
                    Logger.dec_depth()
                else:
                    r = False
            elif type_class == Symbolic("list-of"):
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
            elif type_class == Symbolic("collection-of"):
                subType = type_def.get(1)
                if isinstance(t, Collection):
                    r = True
                    Logger.inc_depth()
                    for e in term:
                        if not self.check(subType, e):
                            r = False
                            break
                    Logger.dec_depth()
                else:
                    r = False
            elif type_class == Symbolic("signed-tuple"):
                if isinstance(term, Tuple):
                    signature = type_def.get(1)
                    min = type_def.get_or_default(Symbolic("min"), Integer(len(signature)))
                    max = type_def.get_or_default(Symbolic("max"), Integer(len(signature)))
                    repeat = type_def.get_or_default(Symbolic("repeat"), Integer(1))
                    tSize = Integer(len(term))
                    repeat_start_idx = len(signature) - repeat.int_value()
                    
                    if tSize >= min and tSize <= max:
                        Logger.inc_depth()
                        r = True
                        for i in range(0, len(signature)):
                            sig_idx = i
                            if i >= signature.size():
                                sig_idx = repeat_start_idx + \
                                    (i - signature.size()) % repeat.getIntValue()
                                if not self.check(signature.get(sig_idx), term.get(i)):
                                    r = False
                                    break
                        Logger.dec_depth()
                    else:
                        r = False
                else:
                    r = False
            elif type_class == Symbolic("key-value-tuple"):
                keyTypeCol = type_def.get(1)
                r = True
                Logger.inc_depth()
                for kvp in keyTypeCol:
                    e = term.get(kvp.get_key())
                    if e is None:
                        r = False
                        break
                    if not self.check(kvp.get_value(), e):
                        r = False
                        break
                Logger.dec_depth()
            elif type_class == Symbolic("enum"):
                r = term in type_def.get(1)
            elif type_class == Symbolic("numerical-range"):
                min = type_def.get_or_default(Symbolic("min"), Infinity.neg())
                max = type_def.get_or_default(Symbolic("max"), Infinity.pos())
                r = False
                if isinstance(term, Numerical):
                    if term >= min and term <= max:
                        r = True
            elif type_class == Symbolic("typed-key-value"):
                r = False
                if isinstance(term, KeyValue):
                    if self.check(type_def[1], term.get_key()) and self.check(type_def[1], term.get_value()):
                        r = True
            elif type_class == Symbolic("or-type"):
                r = False
                for choice in type_def[1]:
                    if self.check(choice, term):
                        r = True
                        break 
            else:
                raise ValueError("#type expression not supported:", type_def)
        elif isinstance(type_def, Symbolic):
            e = Tuple([type_def, term])
            r = self.evaluator.apply(e).bool_value()
        else:
            r = False
            raise ValueError("#type expression not supported:", type_def)

        if r and isinstance(type_def, Tuple):
            constraint = type_def[Symbolic("constraint")]
            if constraint is not None and isinstance(constraint, FunctionReference):
                r = constraint.get_function().apply(term).bool_value()
                # if not r:
                #     if eval.get_verbosity() >= 1:
                #         Logger.incDepth()
                #         Logger.msg("TypeCheck", t, " does not satisfy " + constraint)
                #         Logger.decDepth()

        # if not r:
        #     Logger.msg("TypeCheck", str(term) + " !!  " + str(type_def))
        return r
        
