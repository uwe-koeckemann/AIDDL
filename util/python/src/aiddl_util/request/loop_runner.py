from aiddl_core.tools.logger import Logger


def run_loop(rhandler, request, exec_module):
    while True:
        if rhandler.verbose:
            Logger.msg(rhandler.name, "loop")
            Logger.inc_depth()
        rhandler.satisfy_request(request[1], exec_module)
        if rhandler.verbose:
            Logger.dec_depth()
