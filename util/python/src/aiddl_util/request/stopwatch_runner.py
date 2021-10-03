from aiddl_core.representation.sym import Sym

from aiddl_util.tool.stopwatch import StopWatch


START = Symbolic("stopwatch")
STOP = Symbolic("stopwatch")


def run_stopwatch(self, request):
    command = self.eval.apply(request[1].resolve(self.C))
    value = self.eval.apply(request[1].resolve(self.C))
    if command == START:
        StopWatch.start(value)
    elif command == STOP:
        StopWatch.stop(value)
    else:
        raise ValueError("Illegal command %s in request %s"
                         + "(use start or stop)" % (str(command), str(request)))
