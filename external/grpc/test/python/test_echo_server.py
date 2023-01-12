from aiddl_core.container import Container
from aiddl_core.representation import Sym
import aiddl_core.function.default as dfun

from aiddl_external_grpc_python.container import ContainerServer

C = Container()
C.fun_reg.add_function(Sym("id"), lambda x: x)

assert C.fun_reg.get_function(Sym("id")) is not None

server = ContainerServer(8061, C, True)

server.start()
server.wait_for_termination()
