import aiddl_core.function.eval.higher_order
import aiddl_core.function.eval.misc
import aiddl_core.function.eval.python
import aiddl_core.function.eval.type
import aiddl_core.function.function
from aiddl_core.function.function_registry import FunctionRegistry
from aiddl_core.function.evaluator import Evaluator
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.inf import Inf
from aiddl_core.representation.int import Int
from aiddl_core.representation.keyval import KeyVal
from aiddl_core.representation.list import List
from aiddl_core.representation.num import Num
from aiddl_core.representation.rat import Rat
from aiddl_core.representation.real import Real
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.set import Set
from aiddl_core.representation.str import Str
from aiddl_core.representation.sym import Boolean, Sym
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.var import Var

import aiddl_core.function.eval.numerical as numerical
import aiddl_core.function.eval.misc as misc
import aiddl_core.function.eval.type as types
import aiddl_core.function.eval.logic as logic
import aiddl_core.function.eval.collection as collection
import aiddl_core.function.eval.string as string

import aiddl_core.function as furi


def get_default_function_registry(container):
    freg = FunctionRegistry()

    evaluator = Evaluator(container)
    freg.add_function(furi.EVAL, evaluator)
    freg.add_function(furi.EVAL_REF, aiddl_core.function.eval.misc.EvalReferenceFunction(evaluator))
    freg.add_function(furi.EVAL_ALL_REFS,
                      aiddl_core.function.eval.misc.EvalAllReferencesFunction(evaluator))
    
    freg.add_function(furi.CALL, aiddl_core.function.eval.higher_order.CallFunction(freg))

    freg.add_function(furi.LAMBDA,
                      aiddl_core.function.function.LambdaEvaluator(freg))
    freg.add_function(furi.FUNCTION_LOADER,
                      aiddl_core.function.eval.python.PythonFunctionLoader(freg))
    freg.add_function(furi.FACTORY_LOADER,
                      aiddl_core.function.eval.python.PythonFunctionFactoryLoader(freg))

    freg.add_function(furi.CORE_LANG, lambda x: Sym("python"))

    # org.aiddl.eval.symbolic
    freg.add_function(furi.SYM_CONCAT, lambda x: x[0] + x[1])
    freg.add_function(furi.SYM_SPLIT, lambda x: List([Sym(e) for e in str(x).split(".")]))

    # org.aiddl.eval.string
    freg.add_function(furi.STR_CONCAT, string.StringConcatFunction())

    # org.aiddl.eval.numerical
    freg.add_function(furi.ADD, numerical.Addition())
    freg.add_function(furi.SUB, numerical.Subtraction())
    freg.add_function(furi.MULT, numerical.Multiplication())
    freg.add_function(furi.DIV, numerical.Division())
    freg.add_function(furi.MODULO, numerical.Modulo())
    freg.add_function(furi.GREATER_THAN, lambda x: Boolean.create(x.get(0) > x.get(1)))
    freg.add_function(furi.GREATER_THAN_EQ, lambda x: Boolean.create(x.get(0) >= x.get(1)))
    freg.add_function(furi.LESS_THAN, lambda x: Boolean.create(x.get(0) < x.get(1)))
    freg.add_function(furi.LESS_THAN_EQ, lambda x: Boolean.create(x.get(0) <= x.get(1)))
    freg.add_function(furi.IS_POS, lambda x: Boolean.create(x.is_positive()))
    freg.add_function(furi.IS_NEG, lambda x: Boolean.create(x.is_negative()))
    freg.add_function(furi.IS_ZERO, lambda x: Boolean.create(x.is_zero()))
    freg.add_function(furi.IS_INF, lambda x: Boolean.create(x.is_inf()))
    freg.add_function(furi.IS_INF_POS, lambda x: Boolean.create(x.is_inf_pos()))
    freg.add_function(furi.IS_INF_NEG, lambda x: Boolean.create(x.is_inf_neg()))
    freg.add_function(furi.IS_NAN, lambda x: Boolean.create(x.is_nan()))

    freg.add_function(furi.LET, misc.Let(evaluator))
    freg.add_function(furi.MAP, aiddl_core.function.eval.higher_order.Map(freg))
    freg.add_function(furi.FILTER, aiddl_core.function.eval.higher_order.Filter(freg))
    freg.add_function(furi.REDUCE, aiddl_core.function.eval.higher_order.Reduce(freg))

    freg.add_function(furi.ZIP, collection.Zip(evaluator))
    freg.add_function(furi.MATCH, misc.Match(evaluator))
    freg.add_function(furi.DOMAIN, misc.ExpandDomain(evaluator))

    freg.add_function(furi.NOT, lambda x: Boolean.create(not x.bool))
    freg.add_function(furi.AND, logic.And(evaluator))
    freg.add_function(furi.OR, logic.Or(evaluator))
    freg.add_function(furi.FORALL, logic.Forall(evaluator))
    freg.add_function(furi.EXISTS, logic.Exists(evaluator))
    freg.add_function(furi.IF, logic.If(evaluator))
    freg.add_function(furi.COND, logic.Cond(evaluator))

    freg.add_function(furi.IN, lambda x: Boolean(x[1].contains(x[0])))
    freg.add_function(furi.CONTAINS, lambda x: Boolean(x[0].contains(x[1])))
    freg.add_function(furi.CONTAINS_ALL, lambda x: Boolean(x[0].contains_all(x[1])))
    freg.add_function(furi.CONTAINS_ANY, collection.ContainsAny())
    freg.add_function(furi.ADD_COL, lambda x: x[0].add(x[1]))
    freg.add_function(furi.ADD_COL_ALL, lambda x: x[0].add_all(x[1]))
    freg.add_function(furi.REM_COL, lambda x: x[0].remove(x[1]))
    freg.add_function(furi.REM_COL_ALL, lambda x: x[0].remove_all(x[1]))
    freg.add_function(furi.UNION, collection.Union())
    freg.add_function(furi.CONCAT, collection.Concat())
    freg.add_function(furi.SUM, collection.Sum())
    freg.add_function(furi.CONTAINS_MATCH, collection.ContainsMatch())
    freg.add_function(furi.PUT_ALL, lambda x: x[0].put_all(x[1]))

    freg.add_function(furi.GET_IDX, lambda x: x[1][x[0].int_value])
    freg.add_function(furi.GET_KEY, lambda x: x[1][x[0]])
    freg.add_function(furi.CONTAINS_KEY, lambda x: Boolean.create(x[0].contains_key(x[1])))

    freg.add_function(furi.QUOTE, misc.Quote())
    freg.add_function(furi.EVAL_REF, misc.EvalRef(evaluator))
    freg.add_function(furi.MATCHES, misc.Matches())
    freg.add_function(furi.EQUALS, lambda x: Boolean.create(x[0] == x[1]))
    freg.add_function(furi.NOT_EQUALS, lambda x: Boolean.create(x[0] != x[1]))
    freg.add_function(furi.SUBSTITUTE, misc.SubstitutionFunction())
    freg.add_function(furi.UNIQUE_MAP, lambda x: Boolean.create(x.is_unique_map()) )

    freg.add_function(furi.FIRST, lambda x: x[0])
    freg.add_function(furi.LAST, lambda x: x[-1])
    freg.add_function(furi.KEY, lambda x: x.key)
    freg.add_function(furi.VALUE, lambda x: x.value)

    freg.add_function(furi.SIZE, lambda x: Int(len(x)))

    freg.add_function(furi.TYPE, types.EvalType(freg))
    freg.add_function(furi.TYPE_TERM, lambda x: Boolean.create(True))
    freg.add_function(furi.TYPE_NUMERICAL, lambda x: Boolean.create(isinstance(x, Num)))
    freg.add_function(furi.TYPE_INTEGER, lambda x: Boolean.create(isinstance(x, Int)))
    freg.add_function(furi.TYPE_RATIONAL, lambda x: Boolean.create(isinstance(x, Rat)))
    freg.add_function(furi.TYPE_REAL, lambda x: Boolean.create(isinstance(x, Real)))
    freg.add_function(furi.TYPE_INF, lambda x: Boolean.create(isinstance(x, Inf)))
    freg.add_function(furi.TYPE_SYMBOLIC, lambda x: Boolean.create(isinstance(x, Sym)))
    freg.add_function(furi.TYPE_BOOLEAN, lambda x: Boolean.create(isinstance(x, Boolean)))
    freg.add_function(furi.TYPE_STRING, lambda x: Boolean.create(isinstance(x, Str)))
    freg.add_function(furi.TYPE_VARIABLE, lambda x: Boolean.create(isinstance(x, Var)))
    freg.add_function(furi.TYPE_EREF, lambda x: Boolean.create(isinstance(x, EntRef)))
    freg.add_function(furi.TYPE_FREF, lambda x: Boolean.create(isinstance(x, FunRef)))
    freg.add_function(furi.TYPE_COLLECTION, lambda x: Boolean.create(isinstance(x, Collection)))
    freg.add_function(furi.TYPE_SET, lambda x: Boolean.create(isinstance(x, Set)))
    freg.add_function(furi.TYPE_LIST, lambda x: Boolean.create(isinstance(x, List)))
    freg.add_function(furi.TYPE_TUPLE, lambda x: Boolean.create(isinstance(x, Tuple)))
    freg.add_function(furi.TYPE_KEY_VALUE, lambda x: Boolean.create(isinstance(x, KeyVal)))

    freg.load_def(container)

    return freg
