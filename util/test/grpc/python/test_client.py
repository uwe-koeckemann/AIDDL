import aiddl_core.parser.parser as parser
from aiddl_core.representation.symbolic import Symbolic
from aiddl_network.grpc_function import GrpcFunction

t = parser.parse_term("(1 2 3)")

print(t)

f = GrpcFunction("localhost", 8011, Symbolic("org.aiddl.eval.add"))

print(f.compute(t))
