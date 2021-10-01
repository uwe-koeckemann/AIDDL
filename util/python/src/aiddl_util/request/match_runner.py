from aiddl_core.tools.logger import Logger


def run_match(self, request, exec_module):
    a = request[1].resolve(self.C)
    b = request[2].resolve(self.C)
    sub_request = request[3]
    s = a.match(b)
    if self.verbose:
        Logger.msg(self.name, "match %s to %s -> %s"
                   % (str(a), str(b), str(s)))
    if s is not None:
        self.satisfy_request(sub_request.substitute(s),
                             exec_module)
