import sys
import aiddl_core.parser.parser as aiddl_parser

from aiddl_core.container.container import Container

from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.symbolic import TRUE
from aiddl_core.representation.symbolic import FALSE
from aiddl_core.representation.list import List

from aiddl_core.representation.variable import Variable

from aiddl_core.request.request_handler import RequestHandler

from aiddl_core.function.uri import EVAL
import aiddl_core.function.default as dfun


ASSERT = Symbolic("#assert")


def run(C, evaluator, freg, verbose):
    n_tests = 0
    n_successful = 0

    rHandler = RequestHandler(C, freg)
    # rHandler.verbose = True
    for req in C.get_matching_entries(Variable(),
                                      Symbolic("#assert-request"),
                                      Variable()):
        request_term = req.get_value()[0]
        exec_module = req.get_value()[1].resolve(C)
        rHandler.satisfy_request(request_term, exec_module)

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
        result = evaluator.apply(test)
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
