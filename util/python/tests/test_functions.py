import os

import aiddl_core.function.default
import aiddl_core.unit_test.unit_test_runner as utest
from aiddl_core.container.container import Container

from src.aiddl_util.function.function_loader import load_functions

my_abs_folder = os.path.dirname(os.path.abspath(__file__))
c = Container()
fun_reg = aiddl_core.function.default.get_default_function_registry(c)
load_functions(fun_reg, c)

# def test_requests():
#     test_file = my_abs_folder + "/../../test/test-request.aiddl"
#     utest.run_aiddl_test_file(test_file)


def test_math():
    test_file = my_abs_folder + "/../../test/function/math.aiddl"
    utest.run_aiddl_test_file(test_file, context=(fun_reg, c))


def test_random():
    test_file = my_abs_folder + "/../../test/function/random.aiddl"
    utest.run_aiddl_test_file(test_file, context=(fun_reg, c))
