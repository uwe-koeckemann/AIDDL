from .function import FunctionMixin
from .function import LazyFunctionMixin
from .function import InitializableMixin
from .function import ConfigurableMixin
from .function import InterfaceImplementationMixin
from .evaluator import Evaluator
from .function_registry import FunctionRegistry

from aiddl_core.representation.sym import Sym

EVAL = Sym("org.aiddl.eval")
EVAL_REF = Sym("org.aiddl.eval.eval-ref")
EVAL_ALL_REFS = Sym("org.aiddl.eval.eval-all-refs")

FUNCTION_LOADER = Sym("org.aiddl.eval.load-function")
FACTORY_LOADER = Sym("org.aiddl.eval.load-function-factory")
CORE_LANG = Sym("org.aiddl.eval.core-lang")

# org.aiddl.eval

CALL = Sym("org.aiddl.eval.call")
LAMBDA = Sym("org.aiddl.eval.lambda")
QUOTE = Sym("org.aiddl.eval.quote")
TYPE = Sym("org.aiddl.eval.type")
ZIP = Sym("org.aiddl.eval.zip")
MAP = Sym("org.aiddl.eval.map")
FILTER = Sym("org.aiddl.eval.filter")
REDUCE = Sym("org.aiddl.eval.reduce")
DOMAIN = Sym("org.aiddl.eval.domain")
EVAL_REF = Sym("org.aiddl.eval.eval-ref")
MATCHES = Sym("org.aiddl.eval.matches")
EQUALS = Sym("org.aiddl.eval.equals")
NOT_EQUALS = Sym("org.aiddl.eval.not-equals")
SUBSTITUTE = Sym("org.aiddl.eval.substitute")
GET_MATCHING_ENTRIES = Sym("org.aiddl.eval.get-matching-entries")

FIRST = Sym("org.aiddl.eval.first")
LAST = Sym("org.aiddl.eval.last")
KEY = Sym("org.aiddl.eval.key")
VALUE = Sym("org.aiddl.eval.value")
SIZE = Sym("org.aiddl.eval.size")
GET_IDX = Sym("org.aiddl.eval.get-idx")
GET_KEY = Sym("org.aiddl.eval.get-key")
CONTAINS_KEY = Sym("org.aiddl.eval.contains-key")
IF = Sym("org.aiddl.eval.if")
COND = Sym("org.aiddl.eval.cond")
MATCH = Sym("org.aiddl.eval.match")
LET = Sym("org.aiddl.eval.let")
UNIQUE_MAP = Sym("org.aiddl.eval.is-unique-map")

# org.aiddl.eval.symbolic

SYM_CONCAT = Sym("org.aiddl.eval.symbolic.concat")
SYM_SPLIT = Sym("org.aiddl.eval.symbolic.split")

# org.aiddl.eval.string

STR_CONCAT = Sym("org.aiddl.eval.string.concat")
STR_MOD_NAME = Sym("org.aiddl.eval.string.module-folder")

# org.aiddl.eval.collection

IN = Sym("org.aiddl.eval.collection.in")
CONTAINS = Sym("org.aiddl.eval.collection.contains")
CONTAINS_ALL = Sym("org.aiddl.eval.collection.contains-all")
CONTAINS_ANY = Sym("org.aiddl.eval.collection.contains-any")
ADD_COL = Sym("org.aiddl.eval.collection.add-element")
ADD_COL_ALL = Sym("org.aiddl.eval.collection.add-all")
REM_COL = Sym("org.aiddl.eval.collection.remove")
REM_COL_ALL = Sym("org.aiddl.eval.collection.remove-all")
CONTAINS_MATCH = Sym("org.aiddl.eval.collection.contains-match")
PUT_ALL = Sym("org.aiddl.eval.collection.put-all")
SUM = Sym("org.aiddl.eval.collection.sum")

# org.aiddl.eval.set

UNION = Sym("org.aiddl.eval.set.union")

# org.aiddl.eval.list

CONCAT = Sym("org.aiddl.eval.list.concat")

# org.aiddl.eval.numeric

ADD = Sym("org.aiddl.eval.numerical.add")
SUB = Sym("org.aiddl.eval.numerical.sub")
MULT = Sym("org.aiddl.eval.numerical.mult")
DIV = Sym("org.aiddl.eval.numerical.div")
MODULO = Sym("org.aiddl.eval.numerical.modulo")
GREATER_THAN = Sym("org.aiddl.eval.numerical.greater-than")
GREATER_THAN_EQ = Sym("org.aiddl.eval.numerical.greater-than-eq")
LESS_THAN = Sym("org.aiddl.eval.numerical.less-than")
LESS_THAN_EQ = Sym("org.aiddl.eval.numerical.less-than-eq")
IS_POS = Sym("org.aiddl.eval.numerical.is-positive")
IS_NEG = Sym("org.aiddl.eval.numerical.is-negative")
IS_ZERO = Sym("org.aiddl.eval.numerical.is-zero")
IS_INF = Sym("org.aiddl.eval.numerical.is-infinite")
IS_INF_POS = Sym("org.aiddl.eval.numerical.is-infinite-positive")
IS_INF_NEG = Sym("org.aiddl.eval.numerical.is-infinite-negative")
IS_NAN = Sym("org.aiddl.eval.numerical.is-nan")

# org.aiddl.eval.random

UNIFORM_RANDOM = Sym("org.aiddl.eval.random.uniform.real")
UNIFORM_RANDOM_ELEMENT = Sym("org.aiddl.eval.random.uniform.element")
UNIFORM_RANDOM_INTEGER = Sym("org.aiddl.eval.random.uniform.integer")
NORMAL_RANDOM = Sym("org.aiddl.eval.random.normal")


# org.aiddl.eval.logic

AND = Sym("org.aiddl.eval.logic.and")
OR = Sym("org.aiddl.eval.logic.or")
NOT = Sym("org.aiddl.eval.logic.not")
EXISTS = Sym("org.aiddl.eval.logic.exists")
FORALL = Sym("org.aiddl.eval.logic.forall")

# org.aiddl.type

TYPE_TERM = Sym("org.aiddl.type.term")
TYPE_NUMERICAL = Sym("org.aiddl.type.term.numerical")
TYPE_INTEGER = Sym("org.aiddl.type.term.numerical.integer")
TYPE_RATIONAL = Sym("org.aiddl.type.term.numerical.rational")
TYPE_REAL = Sym("org.aiddl.type.term.numerical.real")
TYPE_INF = Sym("org.aiddl.type.term.numerical.infinity")
TYPE_SYMBOLIC = Sym("org.aiddl.type.term.symbolic")
TYPE_BOOLEAN = Sym("org.aiddl.type.term.symbolic.boolean")
TYPE_STRING = Sym("org.aiddl.type.term.string")
TYPE_VARIABLE = Sym("org.aiddl.type.term.variable")
TYPE_EREF = Sym("org.aiddl.type.term.ref.entry")
TYPE_FREF = Sym("org.aiddl.type.term.ref.function")
TYPE_COLLECTION = Sym("org.aiddl.type.term.collection")
TYPE_SET = Sym("org.aiddl.type.term.collection.set")
TYPE_LIST = Sym("org.aiddl.type.term.collection.list")
TYPE_TUPLE = Sym("org.aiddl.type.term.tuple")
TYPE_KEY_VALUE = Sym("org.aiddl.type.term.key-value")

