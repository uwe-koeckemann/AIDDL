from aiddl_core.function.function import LazyFunctionMixin
from aiddl_core.representation import Term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.keyval import KeyVal

import aiddl_core.function as furi

SELF = Sym("#self")
SELF_ALT = Sym("#arg")


class Evaluator:
    def __init__(self, container):
        self._log_indent = ""
        self._verbose = False

        self._follow_references = False
        self._container = container

    def set_follow_references(self, flag):
        self._follow_references = flag

    def set_verbose(self, flag):
        self._verbose = flag

    def __call__(self, arg):
        if self._verbose:
            print(self._log_indent + str(arg))
            self._log_indent += "  "

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
            elif isinstance(arg, KeyVal):
                result = KeyVal(
                    self(arg.key), self(arg.value))
            elif isinstance(arg, EntRef):
                if self._follow_references:
                    result = self(self._container.resolve_reference(arg))
                else:
                    result = self._container.resolve_reference(arg)
            else:
                result = arg
        else:
            operator = arg.get(0)
            if isinstance(operator, EntRef):
                if isinstance(operator.target, Sym):
                    uri = operator.convert2uri()
                    if self._container._fun_reg.has_function(uri):
                        operator = uri
                    else:
                        operator = self._container.resolve_reference(operator)
                else:
                    operator = self._container.resolve_reference(operator)

            resolved_arguments = []
            processed_op = self(operator)
            is_lazy = False
            if not self._container._fun_reg.has_function(processed_op):
                resolved_arguments.append(processed_op)
            else:
                is_lazy = isinstance(self._container._fun_reg.get_function(operator), LazyFunctionMixin)
            for i in range(1, arg.size()):
                if not is_lazy:
                    if isinstance(arg.get(i), EntRef):
                        res_arg = self._container.resolve_reference(arg.get(i))
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
            if self._container._fun_reg.has_function(processed_op):
                result = self._container._fun_reg.get_function(processed_op)(processed_args)
            else:
                result = processed_args
 
        if self._verbose:
            if operator is None:
                operator = Sym("-")
            self._log_indent = self._log_indent[0:-2]
            print(self._log_indent + str(result) + "//" + str(operator))
        assert isinstance(result, Term)
        return result

