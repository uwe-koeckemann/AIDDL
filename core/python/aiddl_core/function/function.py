from abc import ABC, abstractmethod

from aiddl_core.function import uri as furi
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.sym import Sym


class Function(ABC):
    @abstractmethod
    def __call__(self, args):
        """Apply function to input term."""


class LazyFunction(Function):
    @abstractmethod
    def __call__(self, args):
        """Apply function to input term."""


class InitializableFunction(ABC):
    @abstractmethod
    def initialize(self, args):
        """Initialize function."""


class ConfigureFunction(ABC):
    @abstractmethod
    def configure(self, cfg, freg):
        """Configure function."""


class InterfaceImplementation(ABC):
    @abstractmethod
    def get_interface_uri(self):
        """Get function interface URI."""


SELF = Sym("#self")
SELF_ALT = Sym("#arg")


class NamedFunction:
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


class LambdaEvaluator(LazyFunction):
    NextID = 0

    def __init__(self, freg):
        self.freg = freg
        self.evaluator = freg.get_function(furi.EVAL)

    def __call__(self, arg):
        arg_term = arg[0]
        fun_term = arg[1]

        f = LambdaFunction(arg_term, fun_term, self.evaluator)
        uri = Sym("#lambda_%d" % LambdaEvaluator.NextID)
        LambdaEvaluator.NextID += 1
        self.freg.add_function(uri, f)
        return FunRef(uri, self.freg)


class LambdaFunction:
    def __init__(self, x, f, e):
        self.x = x
        self.f = f
        self.e = e

    def __call__(self, arg):
        s = self.x.match(arg)
        return self.e(self.f.substitute(s))


class InitFunction:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Sym):
            func = self.freg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunRef):
            func = f.get_function()
            uri = f.get_fref()
        else:
            raise ValueError("Not a symbol or function reference: %s" % str(f))

        func.initialize(args)
        return FunRef(uri, self.freg)


class ConfigFunction:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, x):
        f = x[0]
        args = x[1]

        if isinstance(f, Sym):
            func = self.freg.get_function_or_panic(f)
            uri = f
        elif isinstance(f, FunRef):
            func = f.get_function()
            uri = f.get_fref()
        else:
            raise ValueError("Not a symbol or function reference: %s" % str(f))

        func.configure(args, self.freg)
        return FunRef(uri, self.freg)