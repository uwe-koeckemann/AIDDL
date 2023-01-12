import aiddl_core.parser.parser as parser
from aiddl_core.representation import Sym
from aiddl_external_grpc import GrpcFunction

t = parser.parse_term("(1 2 3)")

print(t)

f = GrpcFunction("localhost", 8011, Sym("org.aiddl.eval.numerical.add"))

print(f(t))
