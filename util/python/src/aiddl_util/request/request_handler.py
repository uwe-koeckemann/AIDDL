import aiddl_core.function.uri as furi

from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.list import List
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.reference import Reference

from aiddl_core.request.if_runner import run_if
from aiddl_core.request.list_runner import run_list
from aiddl_core.request.init_runner import run_init
from aiddl_core.request.write_runner import run_write
from aiddl_core.request.call_runner import run_call
from aiddl_core.request.reference_runner import run_reference_request
from aiddl_core.request.match_runner import run_match
from aiddl_core.request.forall_runner import run_forall
from aiddl_core.request.while_runner import run_while
from aiddl_core.request.loop_runner import run_loop
from aiddl_core.request.create_runner import run_create
from aiddl_core.request.print_runner import run_print
from aiddl_core.request.stopwatch_runner import run_stopwatch


IF = Symbolic("if")
WRITE = Symbolic("write")
CREATE = Symbolic("create")
PRINT = Symbolic("print")
STOPWATCH = Symbolic("stopwatch")
INIT = Symbolic("init")
CALL = Symbolic("call")
MATCH = Symbolic("match")
WHILE = Symbolic("while")
FORALL = Symbolic("forall")
LOOP = Symbolic("loop")


class RequestHandler:
    def __init__(self, C, F, name="RequestHandler", verbose=False):
        self.C = C
        self.F = F
        self.verbose = verbose
        self.eval = F.get_function(furi.EVAL)
        self.eval.set_container(C)
        self.name = name
        self.enforce_type_checking = False



    def satisfy_request(self, request, exec_module):
        if isinstance(request, List):
            run_list(self, request, exec_module)
        elif isinstance(request, Reference):
            run_reference_request(self, request, exec_module)
        if isinstance(request, Tuple):
            if request[0] == IF:
                run_if(self, request, exec_module)
            elif request[0] == WRITE:
                run_write(self, request, exec_module)
            elif request[0] == CREATE:
                run_create(self, request, exec_module)
            elif request[0] == PRINT:
                run_print(self, request)
            elif request[0] == STOPWATCH:
                run_stopwatch(self, request)
            elif request[0] == INIT:
                run_init(self, request)
            elif request[0] == CALL:
                run_call(self, request, exec_module)
            elif request[0] == MATCH:
                run_match(self, request, exec_module)
            elif request[0] == FORALL:
                run_forall(self, request, exec_module)
            elif request[0] == WHILE:
                run_while(self, request, exec_module)
            elif request[0] == LOOP:
                run_loop(self, request, exec_module)
            else:
                raise ValueError("Cannot satisfy request: " + str(request))




            
        

        




 
