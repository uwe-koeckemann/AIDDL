import aiddl_core.representation.term as term
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.reference import Reference
# from aiddl_core.function.function_registry import FunctionRegistry

class FunctionReference(term.Term):
    __slots__ = ["_fref", "_f", "_freg"]

    def __init__(self, funName, freg, module=None):
        if isinstance(funName, Reference):
            fun_name = funName.get_ref_target()
            mod_name = funName.get_ref_module()
            super(term.Term, self).__setattr__("_fref", mod_name + fun_name)
            super(term.Term, self).__setattr__("_freg", freg)            
        elif isinstance(funName, Symbolic):
            if module is not None:
                super(term.Term, self).__setattr__("_fref", module + funName)
            else:
                super(term.Term, self).__setattr__("_fref", funName)
            super(term.Term, self).__setattr__("_freg", freg)

        else:
            raise ValueError("%s is not Symbolic or Reference term" % str(funName))
        # elif isinstance(funName, Symbolic) or isinstance(funName, Tuple):
        #     super(term.Term, self).__setattr__("_fref", funName)
        # elif isinstance(funName, Reference):
        #     uri = funName.get_ref_module() + funName.get_ref_target()
        #     super(term.Term, self).__setattr__("_fref", uri)
        # else:
        #     print("Expected one of:")
        #     print("- Symbolic funName (and optional symbolic module).")
        #     print("- Tuple representing lambda function.")
        #     print("- Reference term")
        #     raise AttributeError("funName=\n%s\nmodule (optional)=\n%s"
        #                          % (str(funName), str(module)))

    def __call__(self, arg):
        return self._freg.get_function(self._fref)(arg)

    def get_function(self):
        return self._freg.get_function(self._fref)

    def get_function_or_panic(self):
        return self._freg.get_function_or_panic(self._fref)

    def get_fref(self):
        return self._fref

    def substitute(self, s):
        return FunctionReference(self._fref.substitute(s), self._freg)

    def resolve(self, container):
        return self
        # if isinstance(self._fref, Tuple):
        #     return FunctionReference(self._fref.resolve(container))
        # else:
        #     return self

    def __str__(self):
        return "^%s" % (str(self._fref))

    def __eq__(self, other):
        if isinstance(other, FunctionReference):
            return self._fref == other._fref
        return False

    def __ne__(self, other):
        return not (self == other)

    def __hash__(self):
        return 17*hash(self._fref)

    def unpack(self):
        return str(self)
