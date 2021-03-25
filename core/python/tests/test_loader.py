from aiddl_core.parser.parser import parse_term as parse
import aiddl_core.function.default_functions as dfun

from aiddl_core.container.container import Container
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.integer import Integer
import aiddl_core.parser.parser as parser

from aiddl_core.request.request_handler import RequestHandler

dim = 20
tile_size = 20

screen_size = dim*tile_size

screen = pygame.display.set_mode([screen_size, screen_size])
pygame.init()
pygame.display.set_caption("Region")

DB = Container()
# Parser.parse("../../rpg-world-generation/domains/worlds/world-01/npc.aiddl", DB, ".")
parser.parse("models.aiddl", DB, ".")

freg = dfun.get_default_function_registry(DB)
eval = freg.get_function(Symbolic("org.aiddl.eval"))
