from aiddl_core.tools.logger import Logger


def run_while(rhandler, request, exec_module):
    if rhandler.verbose:
        Logger.msg(rhandler.name, "while")
        Logger.inc_depth()

    condition = request[1].resolve(rhandler.C)

    while rhandler.eval.apply(condition).bool_value():
        rhandler.satisfy_request(request[2], exec_module)
        condition = request[1].resolve(rhandler.C)

    if rhandler.verbose:
        Logger.dec_depth()
