from aiddl_core.container.container import Container
import aiddl_core.function.default as dfun

from aiddl_external_grpc import AiddlServicer

C = Container()
freg = dfun.get_default_function_registry(C)

server = AiddlServicer(8011, freg, True)

server.start()
server.wait_for_termination()
