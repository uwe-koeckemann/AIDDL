package org.aiddl.core.scala.function

import org.aiddl.core.scala.representation.Sym

object DefaultFunctionUri {
    val EVAL = Sym("org.aiddl.eval")
    val EVAL_REF = Sym("org.aiddl.eval.eval-ref")
    val EVAL_ALL_REFS = Sym("org.aiddl.eval.eval-all-refs")
        
    val LOAD_FUNCTION = Sym("org.aiddl.eval.load-function")
    val LOAD_FUNCTION_FACTORY = Sym("org.aiddl.eval.load-function-factory")
        
    val CORE_LANG = Sym("org.aiddl.eval.core-lang")
        
    val CALL_REQUEST = Sym("org.aiddl.eval.call-request")
    val INIT_FUNCTION = Sym("org.aiddl.eval.init-function")
    val CONFIG_FUNCTION = Sym("org.aiddl.eval.config-function")
	
	
	
	/**
	 * org.aiddl.eval
	 */
    val CALL = Sym("org.aiddl.eval.call")
    val LAMBDA = Sym("org.aiddl.eval.lambda")
    val QUOTE = Sym("org.aiddl.eval.quote")
    val TYPE = Sym("org.aiddl.eval.type")
    val ZIP = Sym("org.aiddl.eval.zip")
    val MAP = Sym("org.aiddl.eval.map")
    val FILTER = Sym("org.aiddl.eval.filter")
    val REDUCE = Sym("org.aiddl.eval.reduce")
    val DOMAIN = Sym("org.aiddl.eval.domain")
    val MATCHES = Sym("org.aiddl.eval.matches")
    val EQUALS = Sym("org.aiddl.eval.equals")
    val NOT_EQUALS = Sym("org.aiddl.eval.not-equals")
    val SUBSTITUTE = Sym("org.aiddl.eval.substitute")

    val FIRST = Sym("org.aiddl.eval.first")
    val LAST = Sym("org.aiddl.eval.last")
    val KEY = Sym("org.aiddl.eval.key")
    val VALUE = Sym("org.aiddl.eval.value")
    val SIZE = Sym("org.aiddl.eval.size")
    val GET_IDX = Sym("org.aiddl.eval.get-idx")
    val GET_KEY = Sym("org.aiddl.eval.get-key")
    val IF = Sym("org.aiddl.eval.if")
    val COND = Sym("org.aiddl.eval.cond")
    val MATCH  = Sym("org.aiddl.eval.match")
    val LET    = Sym("org.aiddl.eval.let")
	
	/**
	 * org.aiddl.eval.symbolic
	 */
    val SYM_CONCAT    = Sym("org.aiddl.eval.symbolic.concat")
    val SYM_SPLIT    = Sym("org.aiddl.eval.symbolic.split")
        
	/**
	 * org.aiddl.eval.string
	 */
     val STR_CONCAT    = Sym("org.aiddl.eval.string.concat")
    val STR_MOD_NAME    = Sym("org.aiddl.eval.string.module-folder")
            
	/**
	 * org.aiddl.eval.collection
	 */
    val IN = Sym("org.aiddl.eval.collection.in")
    val CONTAINS = Sym("org.aiddl.eval.collection.contains")
    val CONTAINS_ALL = Sym("org.aiddl.eval.collection.contains-all")
    val CONTAINS_ANY = Sym("org.aiddl.eval.collection.contains-any")
    val ADD_COL = Sym("org.aiddl.eval.collection.add-element")
    val ADD_COL_ALL = Sym("org.aiddl.eval.collection.add-all")
    val REM_COL = Sym("org.aiddl.eval.collection.remove")
    val REM_COL_ALL = Sym("org.aiddl.eval.collection.remove-all")
    val CONTAINS_MATCH = Sym("org.aiddl.eval.collection.contains-match")
    val SUM = Sym("org.aiddl.eval.collection.sum")
	/**
	 * collection & tuple
	 */
    val PUT_ALL = Sym("org.aiddl.eval.collection.put-all")
    val CONTAINS_KEY = Sym("org.aiddl.eval.contains-key")
    val IS_UNIQUE_MAP = Sym("org.aiddl.eval.is-unique-map")
	/**
	 * org.aiddl.eval.set
	 */
    val UNION	= Sym("org.aiddl.eval.set.union")
	/**
	 * org.aiddl.eval.list
	 */
     val CONCAT	= Sym("org.aiddl.eval.list.concat")
	/**
	 * org.aiddl.eval.numeric
	 */
    val ADD = Sym("org.aiddl.eval.numerical.add")
    val SUB = Sym("org.aiddl.eval.numerical.sub")
    val MULT = Sym("org.aiddl.eval.numerical.mult")
    val DIV = Sym("org.aiddl.eval.numerical.div")
    val MODULO = Sym("org.aiddl.eval.numerical.modulo")
        
    val GREATER_THAN = Sym("org.aiddl.eval.numerical.greater-than")
    val GREATER_THAN_EQ = Sym("org.aiddl.eval.numerical.greater-than-eq")
    val LESS_THAN = Sym("org.aiddl.eval.numerical.less-than")
    val LESS_THAN_EQ = Sym("org.aiddl.eval.numerical.less-than-eq")

    val IS_POSITIVE = Sym("org.aiddl.eval.numerical.is-negative")
    val IS_NEGATIVE = Sym("org.aiddl.eval.numerical.is-zero")
    val IS_ZERO = Sym("org.aiddl.eval.numerical.is-zero")
    val IS_NAN = Sym("org.aiddl.eval.numerical.is-nan")
    val IS_INF = Sym("org.aiddl.eval.numerical.is-infinite")
    val IS_INF_POS = Sym("org.aiddl.eval.numerical.is-infinite-positive")
    val IS_INF_NEG = Sym("org.aiddl.eval.numerical.is-infinite-negative")

	/**
	 * org.aiddl.eval.logic
	 */
    val AND = Sym("org.aiddl.eval.logic.and")
    val OR  = Sym("org.aiddl.eval.logic.or")
    val NOT = Sym("org.aiddl.eval.logic.not")
    val EXISTS = Sym("org.aiddl.eval.logic.exists")
    val FORALL = Sym("org.aiddl.eval.logic.forall")
        
    val TYPE_TERM = Sym("org.aiddl.type.term")
    val TYPE_NUMERICAL = Sym("org.aiddl.type.term.numerical")
    val TYPE_INTEGER = Sym("org.aiddl.type.term.numerical.integer")
    val TYPE_RATIONAL = Sym("org.aiddl.type.term.numerical.rational")
    val TYPE_REAL = Sym("org.aiddl.type.term.numerical.real")
    val TYPE_INF = Sym("org.aiddl.type.term.numerical.infinity")
        
    val TYPE_SYMBOLIC = Sym("org.aiddl.type.term.symbolic")
    val TYPE_BOOLEAN = Sym("org.aiddl.type.term.symbolic.boolean")
    val TYPE_STRING = Sym("org.aiddl.type.term.string")
    val TYPE_VARIABLE = Sym("org.aiddl.type.term.variable")
    val TYPE_EREF = Sym("org.aiddl.type.term.ref.entry")
    val TYPE_FREF = Sym("org.aiddl.type.term.ref.function")
        
    val TYPE_COLLECTION = Sym("org.aiddl.type.term.collection")
    val TYPE_SET = Sym("org.aiddl.type.term.collection.set")
    val TYPE_LIST = Sym("org.aiddl.type.term.collection.list")
    val TYPE_TUPLE = Sym("org.aiddl.type.term.tuple")
    val TYPE_KEY_VALUE = Sym("org.aiddl.type.term.key-value")
}
