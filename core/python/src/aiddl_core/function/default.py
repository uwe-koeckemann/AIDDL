from aiddl_core.function.function_registry import FunctionRegistry
from aiddl_core.function.evaluator import Evaluator
from aiddl_core.request.request_handler import RequestHandler

import aiddl_core.function.eval.numerical as numerical
import aiddl_core.function.eval.term as term_eval
import aiddl_core.function.eval.type as type_eval
import aiddl_core.function.eval.symbolic as sym_eval
import aiddl_core.function.eval.logic as logic
import aiddl_core.function.eval.collection as collection
import aiddl_core.function.eval.eval as gen_eval
import aiddl_core.function.eval.string as str_eval

import aiddl_core.function.uri as furi


def get_default_function_registry(C):
    freg = FunctionRegistry()

    evaluator = Evaluator(freg, C)
    freg.add_function(furi.EVAL, evaluator)
    freg.add_function(furi.EVAL_REF, gen_eval.EvalReferenceFunction(evaluator))
    freg.add_function(furi.EVAL_ALL_REFS,
                      gen_eval.EvalAllReferencesFunction(evaluator))
    
    freg.add_function(furi.CALL, term_eval.CallFunction(freg))
    freg.add_function(furi.CALL_REQUEST,
                      gen_eval.CallRequestFunction(C, RequestHandler(C, freg)))
    freg.add_function(furi.LAMBDA,
                      gen_eval.LambdaEvaluator(freg))
    freg.add_function(furi.FUNCTION_LOADER,
                      gen_eval.PythonFunctionLoader(freg))
    freg.add_function(furi.FACTORY_LOADER,
                      gen_eval.PythonFunctionFactoryLoader(freg))

    freg.add_function(furi.CORE_LANG,
                      gen_eval.CoreLang())
    

    # org.aiddl.eval.symbolic
    freg.add_function(furi.SYM_CONCAT, sym_eval.Concat())
    freg.add_function(furi.SYM_SPLIT, sym_eval.Split())

    # org.aiddl.eval.string
    freg.add_function(furi.STR_CONCAT, str_eval.StringConcatFunction())


    # org.aiddl.eval.numerical
    freg.add_function(furi.ADD, numerical.Addition())
    freg.add_function(furi.SUB, numerical.Subtraction())
    freg.add_function(furi.MULT, numerical.Multiplication())
    freg.add_function(furi.DIV, numerical.Division())
    freg.add_function(furi.MODULO, numerical.Modulo())
    freg.add_function(furi.GREATER_THAN, numerical.GreaterThan())
    freg.add_function(furi.GREATER_THAN_EQ, numerical.GreaterOrEquals())
    freg.add_function(furi.LESS_THAN, numerical.LessThan())
    freg.add_function(furi.LESS_THAN_EQ, numerical.LessOrEquals())

    freg.add_function(furi.LET, term_eval.Let(evaluator))
    freg.add_function(furi.MAP, term_eval.Map(freg))
    freg.add_function(furi.FILTER, term_eval.Filter(freg))
    freg.add_function(furi.REDUCE, term_eval.Reduce(freg))

    freg.add_function(furi.SIGNATURE,
                      term_eval.Signature(evaluator))
    freg.add_function(furi.ZIP, term_eval.Zip(evaluator))
    freg.add_function(furi.MATCH, term_eval.Match(evaluator))
    freg.add_function(furi.DOMAIN, term_eval.ExpandDomain(evaluator))

    freg.add_function(furi.NOT, logic.Not())
    freg.add_function(furi.AND, logic.And(evaluator))
    freg.add_function(furi.OR, logic.Or(evaluator))
    freg.add_function(furi.FORALL, logic.Forall(evaluator))
    freg.add_function(furi.EXISTS, logic.Exists(evaluator))
    freg.add_function(furi.IF, logic.If(evaluator))
    freg.add_function(furi.COND, logic.Cond(evaluator))

    freg.add_function(furi.IN, collection.InCollection())
    freg.add_function(furi.CONTAINS, collection.Contains())
    freg.add_function(furi.CONTAINS_ALL, collection.ContainsAll())
    freg.add_function(furi.CONTAINS_ANY, collection.ContainsAny())
    freg.add_function(furi.ADD_COL, collection.Add())
    freg.add_function(furi.ADD_COL_ALL, collection.AddAll())
    freg.add_function(furi.REM_COL, collection.Remove())
    freg.add_function(furi.REM_COL_ALL, collection.RemoveAll())
    freg.add_function(furi.UNION, collection.Union())
    freg.add_function(furi.CONCAT, collection.Concat())
    freg.add_function(furi.SUM, collection.Sum())
    freg.add_function(furi.CONTAINS_MATCH, collection.ContainsMatch())
    freg.add_function(furi.PUT_ALL, collection.PutAll())

    freg.add_function(furi.GET_IDX, term_eval.AtIndex())
    freg.add_function(furi.GET_KEY, term_eval.AtKey())
    freg.add_function(furi.CONTAINS_KEY, term_eval.ContainsKey())

    freg.add_function(furi.QUOTE, term_eval.Quote())
    freg.add_function(furi.EVAL_REF, term_eval.EvalRef(evaluator))
    freg.add_function(furi.MATCHES, term_eval.Matches())
    freg.add_function(furi.EQUALS, term_eval.Equals())
    freg.add_function(furi.NOT_EQUALS, term_eval.NotEquals())
    freg.add_function(furi.SUBSTITUTE, term_eval.SubstitutionFunction())
    freg.add_function(furi.GET_MATCHING_ENTRIES,
                      gen_eval.GetMatchingEntries(C))

    freg.add_function(furi.FIRST, term_eval.First())
    freg.add_function(furi.LAST, term_eval.Last())
    freg.add_function(furi.KEY, term_eval.GetKey())
    freg.add_function(furi.VALUE, term_eval.GetValue())

    freg.add_function(furi.SIZE, term_eval.Size())

    freg.add_function(furi.TYPE, type_eval.EvalType(freg))
    freg.add_function(furi.TYPE_TERM, type_eval.IsTerm())
    freg.add_function(furi.TYPE_NUMERICAL, type_eval.IsNumerical())
    freg.add_function(furi.TYPE_INTEGER, type_eval.IsInteger())
    freg.add_function(furi.TYPE_RATIONAL, type_eval.IsRational())
    freg.add_function(furi.TYPE_REAL, type_eval.IsReal())
    freg.add_function(furi.TYPE_INF, type_eval.IsInfinity())
    freg.add_function(furi.TYPE_SYMBOLIC, type_eval.IsSymbolic())
    freg.add_function(furi.TYPE_BOOLEAN, type_eval.IsBoolean())
    freg.add_function(furi.TYPE_STRING, type_eval.IsString())
    freg.add_function(furi.TYPE_VARIABLE, type_eval.IsVariable())
    freg.add_function(furi.TYPE_REF, type_eval.IsReference())
    freg.add_function(furi.TYPE_FREF, type_eval.IsFunctionReference())
    freg.add_function(furi.TYPE_COLLECTION, type_eval.IsCollection())
    freg.add_function(furi.TYPE_SET, type_eval.IsSet())
    freg.add_function(furi.TYPE_LIST, type_eval.IsList())
    freg.add_function(furi.TYPE_TUPLE, type_eval.IsTuple())
    freg.add_function(furi.TYPE_KEY_VALUE, type_eval.IsKeyValue())

    freg.load_def(C)

    return freg
