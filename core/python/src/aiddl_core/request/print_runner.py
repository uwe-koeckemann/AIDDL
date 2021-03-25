from aiddl_core.tools.logger import Logger


def run_print(rhandler, request, exec_module):
    name = rhandler.eval.compute(request[1].resolve(rhandler.C))
    value = rhandler.eval.compute(request[2].resolve(rhandler.C))
    Logger.msg(str(name).replace('"', ""), str(value))
