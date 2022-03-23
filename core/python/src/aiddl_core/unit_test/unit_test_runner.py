
import aiddl_core.parser.parser as aiddl_parser

from aiddl_core.container import Container

from aiddl_core.representation import Sym
from aiddl_core.representation.sym import TRUE
from aiddl_core.representation import List
from aiddl_core.function.uri import EVAL

ASSERT = Sym("#assert")


def run(c, evaluator, verbose):
    n_tests = 0
    n_successful = 0

    tests = c.get_matching_entries(None, ASSERT, None)
    for test in tests:
        p, t = run_single_test(str(test.get_name()), test.get_value(), evaluator, c.fun_reg, verbose)
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


def run_aiddl_test_file(fname, context=None):
    return run_aiddl_test_files([fname], context=context)


def run_aiddl_test_files(fnames, context=None):
    if context is None:
        c = Container()
    else:
        c = context

    for fname in fnames:
        aiddl_parser.parse(fname, c, ".")

    evaluator = c.fun_reg.get_function(EVAL)
    evaluator.set_container(c)
    result = run(c, evaluator, True)
    print("Test result: %d/%d" % (result[0], result[1]))
    return result
