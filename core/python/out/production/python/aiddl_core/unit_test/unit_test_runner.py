
import aiddl_core.parser.parser as aiddl_parser

from aiddl_core.container.container import Container

from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import TRUE
from aiddl_core.representation.list import List

from aiddl_core.representation.var import Var


from aiddl_core.function.uri import EVAL
import aiddl_core.function.default as dfun


ASSERT = Sym("#assert")


def run(C, evaluator, freg, verbose):
    n_tests = 0
    n_successful = 0

    tests = C.get_matching_entries(None, ASSERT, None)
    for test in tests:
        p, t = run_single_test(str(test.get_name()), test.get_value(), evaluator, freg, verbose)
        n_successful += p
        n_tests += t
    return (n_successful, n_tests)


def run_single_test(label, test, evaluator, freg, verbose):
    if isinstance(test, List):
        n_passed = 0
        n_total = 0
        for sub_test in test:
            p, t = run_single_test(label, sub_test, evaluator, freg, verbose)
            n_passed += p
            n_total += t
        return n_passed, n_total
    else:
        result = evaluator(test)
        if result == TRUE:
            if verbose:
                print("Test %s: ok %s" % (label, test))
            return 1, 1
        else:
            print("Test %s: FAILURE %s" % (label, test))
            print("Result:", result, "->", type(result))
            exit()
            return 0, 1


def run_aiddl_test_file(fname):
    return run_aiddl_test_files([fname])


def run_aiddl_test_files(fnames):
    C = Container()
    freg = dfun.get_default_function_registry(C)

    for fname in fnames:
        aiddl_parser.parse(fname, C, freg, ".")

    evaluator = freg.get_function(EVAL)
    evaluator.set_container(C)
    result = run(C, evaluator, freg, True)
    print("Test result: %d/%d" % (result[0], result[1]))
    return result
