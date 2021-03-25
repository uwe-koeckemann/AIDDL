from aiddl_core.tools.logger import Logger


def run_init(rhandler, request):
    name = rhandler.eval.apply(request[1].resolve(rhandler.C))
    arg = rhandler.eval.apply(request[2].resolve(rhandler.C))
    if rhandler.verbose:
        Logger.msg(rhandler.name, "init " + str(name))
    f = rhandler.F.get_function(name)
    if f is None:
        raise ValueError("Function not registered: " + str(name)
                         + " request: " + str(request))
    f.initialize(arg)
