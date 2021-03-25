import aiddl_core_path_fix  # noqa
import aiddl_core.unit_test.unit_test_runner as utest

test_file = "../../test/test.aiddl"
utest.run_aiddl_test_file(test_file)

test_file = "../../test/test-types.aiddl"
utest.run_aiddl_test_file(test_file)

test_file = "../../test/test-request.aiddl"
utest.run_aiddl_test_file(test_file)

test_file = "../../test/eval/math/test-math.aiddl"
utest.run_aiddl_test_file(test_file)

test_file = "../../test/eval/random/test-random.aiddl"
utest.run_aiddl_test_file(test_file)
