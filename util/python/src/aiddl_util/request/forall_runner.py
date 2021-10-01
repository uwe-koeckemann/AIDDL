from aiddl_core.tools.logger import Logger
from aiddl_core.tools.combo_iterator import ComboIterator
from aiddl_core.representation.substitution import Substitution


def run_forall(rhandler, request, exec_module):
    domain_map = request[1].resolve(rhandler.C)
    sub_request = request[2]
    variables = []
    choices = []
    for var_choice in domain_map:
        variables.append(var_choice.get_key())
        domain = rhandler.eval.apply(var_choice.get_value())
        domain_list = []
        for v in domain:
            domain_list.append(v)
        # domain.add_all_to(domain_list)
        choices.append(domain_list)
    if rhandler.verbose:
        Logger.msg(rhandler.name, "forall choices: " + str(choices))
    combos = ComboIterator(choices)
    s = Substitution()
    for combo in combos:
        for i in range(len(combo)):
            s.add(variables[i], combo[i])

        sub_req_inst = sub_request.substitute(s)
        if rhandler.verbose:
            Logger.msg(rhandler.name, "calling: " + str(sub_req_inst))
            Logger.inc_depth()
        rhandler.satisfy_request(sub_req_inst, exec_module)
        if rhandler.verbose:
            Logger.dec_depth()
