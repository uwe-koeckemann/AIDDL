import aiddl_core.representation.term as term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.entref import EntRef


class FunRef(term.Term):
    __slots__ = ["_fref", "_f", "_freg"]

    def __init__(self, fun_uri, fun_reg, module=None):
        if isinstance(fun_uri, EntRef):
            fun_name = fun_uri.target
            mod_name = fun_uri.module
            super(term.Term, self).__setattr__("_fref", mod_name + fun_name)
            super(term.Term, self).__setattr__("_freg", fun_reg)
        elif isinstance(fun_uri, Sym):
            if module is not None:
                super(term.Term, self).__setattr__("_fref", module + fun_uri)
            else:
                super(term.Term, self).__setattr__("_fref", fun_uri)
            super(term.Term, self).__setattr__("_freg", fun_reg)

        else:
            raise ValueError("%s is not Symbolic or Reference term" % str(fun_uri))

    def __call__(self, arg):
        """ Call the referenced function on a given argument

        :param arg: term arguments to the function
        :return: result of function application
        """
        return self._freg.get_function(self._fref)(arg)

    @property
    def function(self):
        """ Get the function referenced by this term

        :return: function
        """
        return self._freg.get_function(self._fref)

    def get_function_or_panic(self):
        """ Get function if it exists or raise an exception

        :return: function
        """
        return self._freg.get_function_or_panic(self._fref)

    @property
    def function_uri(self):
        return self._fref

    def substitute(self, s):
        return FunRef(self._fref.substitute(s), self._freg)

    def resolve(self, container):
        return self

    def __str__(self):
        return "^%s" % (str(self._fref))

    def __eq__(self, other):
        if isinstance(other, FunRef):
            return self._fref == other._fref
        return False

    def __ne__(self, other):
        return not (self == other)

    def __hash__(self):
        return 17*hash(self._fref)

    def unpack(self):
        return str(self)
