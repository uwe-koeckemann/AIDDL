from aiddl_core.representation.sym import Sym
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.entref import EntRef


def out2ref(out, C, exec_module):
    if isinstance(out, Sym):
        return EntRef(out, exec_module)
    elif isinstance(out, Tuple) and len(out) == 2:
        return EntRef(out[0], out[1].resolve(C))
    raise "Illegal Argument:\nBad format: " + str(out) \
        + "\nTerm used to describe function output should" + \
        "either be symbolic (target in working module) or" +\
        " tuple of size two (reference target and module resp.)."
