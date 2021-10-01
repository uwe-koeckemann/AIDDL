import importlib

from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.sym import Sym


class PythonFunctionLoader:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, x):
        name = x[Sym("name")]
        module = x[Sym("module")]
        py_class = x[Sym("pyclass")]
        py_module = x[Sym("pymodule")]

        imp = importlib.import_module(str(py_module))
        instance = getattr(imp, str(py_class))()
        uri = module + name

        self.freg.add_function(uri, instance)


class PythonFunctionFactoryLoader:
    def __init__(self, freg):
        self.freg = freg

    def __call__(self, x):
        name = x[Sym("name")]
        module = x[Sym("module")]
        py_class = x[Sym("pyclass")]
        py_module = x[Sym("pymodule")]

        imp = importlib.import_module(str(py_module))
        constructor = getattr(imp, str(py_class))

        factory = PythonFunctionFactory(constructor, self.freg)
        uri = module + name

        self.freg.add_function(uri, factory)
        return FunRef(uri, self.freg)


class PythonFunctionFactory:
    def __init__(self, constructor, freg):
        self.constructor = constructor
        self.freg = freg

    def __call__(self, x):
        uri = None
        init = None
        config = None
        if isinstance(x, Sym):
            uri = x
        else:
            uri = x[0]
            init = x[Sym("init")]
            config = x[Sym("config")]

        f = self.constructor()

        if init is not None:
            f.initialize(init)

        if config is not None:
            f.configure(config, self.freg)

        self.freg.add_function(uri, f)
        return FunRef(uri, self.freg)