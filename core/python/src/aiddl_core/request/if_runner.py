from aiddl_core.tools.logger import Logger


def run_if(rhandler, request, exec_module):
    condition = request.get(1).resolve(rhandler.C)
    condition_computed = rhandler.eval.apply(condition)
    if rhandler.verbose:
        Logger.msg(rhandler.name, "if " + str(condition))
        Logger.msg(rhandler.name, "-> " + str(condition_computed))
    if condition_computed.bool_value():
        if rhandler.verbose:
            Logger.msg(rhandler.name, "then: " + str(request.get(2)))
        rhandler.satisfy_request(request.get(2), exec_module)
    else:
        if rhandler.verbose:
            Logger.msg(rhandler.name, "else: " + str(request.get(3)))
        rhandler.satisfy_request(request.get(3), exec_module)
