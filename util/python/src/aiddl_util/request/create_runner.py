from aiddl_core.tools.logger import Logger
from aiddl_core.container.container import Entry
from aiddl_core.representation.funref import FunRef


def run_create(rhandler, request, exec_module):
    entry_term = rhandler.eval.apply(request[1])

    if rhandler.verbose:
        Logger.msg(rhandler.name, "create %s" % (str(entry_term)))

    e_type = entry_term[0]
    e_name = entry_term[1]
    e_value = entry_term[2]

    if rhandler.enforce_type_checking:
        f_t_check = e_type.get_function(rhandler.F)

        if not isinstance(e_type, FunRef):
            print("Request:", request)
            print("Type   :", e_type)
            print("Name   :", e_name)
            print("Value  :", e_value)
            raise ValueError("Type is not a function reference:", e_type)
        elif f_t_check is None:
            print("Request:", request)
            print("Type   :", e_type)
            print("Name   :", e_name)
            print("Value  :", e_value)
            raise ValueError("Function not registered:", e_type)
        type_sat = f_t_check.apply(e_value)
        if not type_sat.bool_value():
            print("Request:", request)
            print("Type   :", e_type)
            print("Name   :", e_name)
            print("Value  :", e_value)
            rhandler.eval.set_verbose(2)
            f_t_check.apply(e_value)
            raise ValueError("Value check failure.")
    e = Entry(e_type, e_name, e_value)
    rhandler.C.set_entry(e, module=exec_module)
