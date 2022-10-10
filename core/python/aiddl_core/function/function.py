from abc import ABC, abstractmethod

import aiddl_core.function as function
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.sym import Sym

SELF = Sym("#self")
SELF_ALT = Sym("#arg")


class FunctionMixin:
    @abstractmethod
    def __call__(self, args):
        """Apply function to input term."""


class LazyFunctionMixin:
    pass


class InitializableMixin:
    @abstractmethod
    def initialize(self, args):
        """Initialize function."""


class ConfigurableMixin:
    @abstractmethod
    def configure(self, cfg, freg):
        """Configure function."""


class InterfaceImplementationMixin:
    @abstractmethod
    def get_interface_uri(self):
        """Get function interface URI."""


class NamedFunction(FunctionMixin):
    def __init__(self, name, f, e, args=None):
        self.name = name
        self.f = f
        self.e = e
        self.args = args

    def __call__(self, arg):
        if self.args is None:
            s = Substitution()
            s.add(SELF, arg)
            s.add(SELF_ALT, arg)
        else:
            s = self.args.match(arg)
        return self.e(self.f.substitute(s))


class LambdaEvaluator(FunctionMixin, LazyFunctionMixin):
    NextID = 0

    def __init__(self, fun_reg):
        self._fun_reg = fun_reg
        self._evaluator = fun_reg.get_function(function.EVAL)

    def __call__(self, arg):
        arg_term = arg[0]
        fun_term = arg[1]

        f = LambdaFunction(arg_term, fun_term, self._evaluator)
        uri = Sym("#lambda_%d" % LambdaEvaluator.NextID)
        LambdaEvaluator.NextID += 1
        self._fun_reg.add_function(uri, f)
        return FunRef(uri, self._fun_reg)


class LambdaFunction(FunctionMixin):
    def __init__(self, x, f, e):
        self.x = x
        self.f = f
        self.e = e

    def __call__(self, arg):
        s = self.x.match(arg)
        return self.e(self.f.substitute(s))


class InitFunction(FunctionMixin):
    def __init__(self, fun_reg):
        self._fun_reg = fun_reg

    def __call__(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Sym):
            func = self._fun_reg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunRef):
            func = f.function
            uri = f.function_uri
        else:
            raise ValueError(f"Not a symbol or function reference: {f}")

        func.initialize(args)
        return FunRef(uri, self._fun_reg)


class ConfigFunction(FunctionMixin):
    def __init__(self, fun_reg):
        self._fun_reg = fun_reg

    def __call__(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Sym):
            func = self._fun_reg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunRef):
            func = f.function
            uri = f.function_uri
        else:
            raise ValueError(f"Not a symbol or function reference: {f}")

        func.configure(args, self._fun_reg)
        return FunRef(uri, self._fun_reg)
