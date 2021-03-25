from aiddl_core.representation.symbolic import Symbolic

EVAL = Symbolic("org.aiddl.eval")
EVAL_REF = Symbolic("org.aiddl.eval.eval-ref")
EVAL_ALL_REFS = Symbolic("org.aiddl.eval.eval-all-refs")

FUNCTION_LOADER = Symbolic("org.aiddl.eval.load-function")
FACTORY_LOADER = Symbolic("org.aiddl.eval.load-function-factory")
CORE_LANG = Symbolic("org.aiddl.eval.core-lang")

# org.aiddl.eval

CALL = Symbolic("org.aiddl.eval.call")
CALL_REQUEST = Symbolic("org.aiddl.eval.call-request")
LAMBDA = Symbolic("org.aiddl.eval.lambda")
QUOTE = Symbolic("org.aiddl.eval.quote")
TYPE = Symbolic("org.aiddl.eval.type")
ZIP = Symbolic("org.aiddl.eval.zip")
MAP = Symbolic("org.aiddl.eval.map")
FILTER = Symbolic("org.aiddl.eval.filter")
REDUCE = Symbolic("org.aiddl.eval.reduce")
DOMAIN = Symbolic("org.aiddl.eval.domain")
EVAL_REF = Symbolic("org.aiddl.eval.eval-ref")
SIGNATURE = Symbolic("org.aiddl.eval.signature")
MATCHES = Symbolic("org.aiddl.eval.matches")
EQUALS = Symbolic("org.aiddl.eval.equals")
NOT_EQUALS = Symbolic("org.aiddl.eval.not-equals")
SUBSTITUTE = Symbolic("org.aiddl.eval.substitute")
GET_MATCHING_ENTRIES = Symbolic("org.aiddl.eval.get-matching-entries")

FIRST = Symbolic("org.aiddl.eval.first")
LAST = Symbolic("org.aiddl.eval.last")
KEY = Symbolic("org.aiddl.eval.key")
VALUE = Symbolic("org.aiddl.eval.value")
SIZE = Symbolic("org.aiddl.eval.size")
GET_IDX = Symbolic("org.aiddl.eval.get-idx")
GET_KEY = Symbolic("org.aiddl.eval.get-key")
CONTAINS_KEY = Symbolic("org.aiddl.eval.contains-key")
IF = Symbolic("org.aiddl.eval.if")
COND = Symbolic("org.aiddl.eval.cond")
MATCH = Symbolic("org.aiddl.eval.match")
LET = Symbolic("org.aiddl.eval.let")

# org.aiddl.eval.symbolic

SYM_CONCAT = Symbolic("org.aiddl.eval.symbolic.concat")
SYM_SPLIT = Symbolic("org.aiddl.eval.symbolic.split")

# org.aiddl.eval.string

STR_CONCAT = Symbolic("org.aiddl.eval.string.concat")
STR_MOD_NAME = Symbolic("org.aiddl.eval.string.module-folder")

# org.aiddl.eval.collection

IN = Symbolic("org.aiddl.eval.collection.in")
CONTAINS = Symbolic("org.aiddl.eval.collection.contains")
CONTAINS_ALL = Symbolic("org.aiddl.eval.collection.contains-all")
CONTAINS_ANY = Symbolic("org.aiddl.eval.collection.contains-any")
ADD_COL = Symbolic("org.aiddl.eval.collection.add-element")
ADD_COL_ALL = Symbolic("org.aiddl.eval.collection.add-all")
REM_COL = Symbolic("org.aiddl.eval.collection.remove")
REM_COL_ALL = Symbolic("org.aiddl.eval.collection.remove-all")
CONTAINS_MATCH = Symbolic("org.aiddl.eval.collection.contains-match")
SUM = Symbolic("org.aiddl.eval.collection.sum")

# org.aiddl.eval.set

UNION = Symbolic("org.aiddl.eval.set.union")

# org.aiddl.eval.list

CONCAT = Symbolic("org.aiddl.eval.list.concat")

# org.aiddl.eval.numeric

ADD = Symbolic("org.aiddl.eval.numerical.add")
SUB = Symbolic("org.aiddl.eval.numerical.sub")
MULT = Symbolic("org.aiddl.eval.numerical.mult")
DIV = Symbolic("org.aiddl.eval.numerical.div")
MODULO = Symbolic("org.aiddl.eval.numerical.modulo")

GREATER_THAN = Symbolic("org.aiddl.eval.numerical.greater-than")
GREATER_THAN_EQ = Symbolic("org.aiddl.eval.numerical.greater-than-eq")
LESS_THAN = Symbolic("org.aiddl.eval.numerical.less-than")
LESS_THAN_EQ = Symbolic("org.aiddl.eval.numerical.less-than-eq")


# org.aiddl.eval.random

UNIFORM_RANDOM = Symbolic("org.aiddl.eval.random.uniform.real")
UNIFORM_RANDOM_ELEMENT = Symbolic("org.aiddl.eval.random.uniform.element")
UNIFORM_RANDOM_INTEGER = Symbolic("org.aiddl.eval.random.uniform.integer")
NORMAL_RANDOM = Symbolic("org.aiddl.eval.random.normal")


# org.aiddl.eval.logic

AND = Symbolic("org.aiddl.eval.logic.and")
OR = Symbolic("org.aiddl.eval.logic.or")
NOT = Symbolic("org.aiddl.eval.logic.not")
EXISTS = Symbolic("org.aiddl.eval.logic.exists")
FORALL = Symbolic("org.aiddl.eval.logic.forall")

TYPE_TERM = Symbolic("org.aiddl.term")
TYPE_NUMERICAL = Symbolic("org.aiddl.term.numerical")
TYPE_INTEGER = Symbolic("org.aiddl.term.numerical.integer")
TYPE_RATIONAL = Symbolic("org.aiddl.term.numerical.rational")
TYPE_REAL = Symbolic("org.aiddl.term.numerical.real")
TYPE_INF = Symbolic("org.aiddl.term.numerical.infinity")

TYPE_SYMBOLIC = Symbolic("org.aiddl.term.symbolic")
TYPE_BOOLEAN = Symbolic("org.aiddl.term.symbolic.boolean")
TYPE_STRING = Symbolic("org.aiddl.term.string")
TYPE_VARIABLE = Symbolic("org.aiddl.term.variable")
TYPE_REF = Symbolic("org.aiddl.term.reference")
TYPE_FREF = Symbolic("org.aiddl.term.fun-ref")

TYPE_COLLECTION = Symbolic("org.aiddl.term.collection")
TYPE_SET = Symbolic("org.aiddl.term.collection.set")
TYPE_LIST = Symbolic("org.aiddl.term.collection.list")
TYPE_TUPLE = Symbolic("org.aiddl.term.tuple")
TYPE_KEY_VALUE = Symbolic("org.aiddl.term.key-value")
