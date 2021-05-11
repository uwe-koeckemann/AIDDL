import os

import aiddl_core_path_fix # noqa
import aiddl_core.unit_test.unit_test_runner as utest

my_abs_folder = os.path.dirname(os.path.abspath(__file__))

def test_core():
    test_file = my_abs_folder + "/../../test/test.aiddl"
    utest.run_aiddl_test_file(test_file)

def test_types():
    test_file = my_abs_folder + "/../../test/test-types.aiddl"
    utest.run_aiddl_test_file(test_file)

def test_requests():
    test_file = my_abs_folder + "/../../test/test-request.aiddl"
    utest.run_aiddl_test_file(test_file)

def test_math():
    test_file = my_abs_folder + "/../../test/eval/math/test-math.aiddl"
    utest.run_aiddl_test_file(test_file)

def test_random():
    test_file = my_abs_folder + "/../../test/eval/random/test-random.aiddl"
    utest.run_aiddl_test_file(test_file)
