from aiddl_core.tools.logger import Logger


def run_list(rhandler, request, exec_module):
    if rhandler.verbose:
        Logger.msg(rhandler.name, "begin list" + str(request))
        Logger.inc_depth()
    for sub_request in request:
        rhandler.satisfy_request(sub_request, exec_module)
    if rhandler.verbose:
        Logger.dec_depth()
        Logger.msg(rhandler.name, "end list")
