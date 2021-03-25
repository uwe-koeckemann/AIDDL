from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.reference import Reference


def out2ref(out, C, exec_module):
    if isinstance(out, Symbolic):
        return Reference(out, exec_module)
    elif isinstance(out, Tuple) and len(out) == 2:
        return Reference(out[0], out[1].resolve(C))
    raise "Illegal Argument:\nBad format: " + str(out) \
        + "\nTerm used to describe function output should" + \
        "either be symbolic (target in working module) or" +\
        " tuple of size two (reference target and module resp.)."
