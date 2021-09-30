package org.aiddl.core.function;

import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class Uri {
    public static final SymbolicTerm EVAL = Term.sym("org.aiddl.eval");
    public static final SymbolicTerm EVAL_REF = Term.sym("org.aiddl.eval.eval-ref");
    public static final SymbolicTerm EVAL_ALL_REFS = Term.sym("org.aiddl.eval.eval-all-refs");

    public static final SymbolicTerm LOAD_JAVA = Term.sym("org.aiddl.eval.load-function");
    public static final SymbolicTerm LOAD_JAVA_FACT = Term.sym("org.aiddl.eval.load-function-factory");

    public static final SymbolicTerm CORE_LANG = Term.sym("org.aiddl.eval.core-lang");


    public static final SymbolicTerm INIT_FUNCTION = Term.sym("org.aiddl.eval.init-function");
    public static final SymbolicTerm CONFIG_FUNCTION = Term.sym("org.aiddl.eval.config-function");

    /**
     * org.aiddl.eval
     */
    public static final SymbolicTerm CALL = Term.sym("org.aiddl.eval.call");
    public static final SymbolicTerm LAMBDA = Term.sym("org.aiddl.eval.lambda");
    public static final SymbolicTerm QUOTE = Term.sym("org.aiddl.eval.quote");
    public static final SymbolicTerm TYPE = Term.sym("org.aiddl.eval.type");
    public static final SymbolicTerm ZIP = Term.sym("org.aiddl.eval.zip");
    public static final SymbolicTerm MAP = Term.sym("org.aiddl.eval.map");
    public static final SymbolicTerm FILTER = Term.sym("org.aiddl.eval.filter");
    public static final SymbolicTerm REDUCE = Term.sym("org.aiddl.eval.reduce");
    public static final SymbolicTerm DOMAIN = Term.sym("org.aiddl.eval.domain");
    public static final SymbolicTerm SIGNATURE = Term.sym("org.aiddl.eval.signature");
    public static final SymbolicTerm MATCHES = Term.sym("org.aiddl.eval.matches");
    public static final SymbolicTerm EQUALS = Term.sym("org.aiddl.eval.equals");
    public static final SymbolicTerm NOT_EQUALS = Term.sym("org.aiddl.eval.not-equals");
    public static final SymbolicTerm SUBSTITUTE = Term.sym("org.aiddl.eval.substitute");
    public static final SymbolicTerm GET_MATCHING_ENTRIES = Term.sym("org.aiddl.eval.get-matching-entries");

    public static final SymbolicTerm FIRST = Term.sym("org.aiddl.eval.first");
    public static final SymbolicTerm LAST = Term.sym("org.aiddl.eval.last");
    public static final SymbolicTerm KEY = Term.sym("org.aiddl.eval.key");
    public static final SymbolicTerm VALUE = Term.sym("org.aiddl.eval.value");
    public static final SymbolicTerm SIZE = Term.sym("org.aiddl.eval.size");
    public static final SymbolicTerm GET_IDX = Term.sym("org.aiddl.eval.get-idx");
    public static final SymbolicTerm GET_KEY = Term.sym("org.aiddl.eval.get-key");
    public static final SymbolicTerm IF = Term.sym("org.aiddl.eval.if");
    public static final SymbolicTerm COND = Term.sym("org.aiddl.eval.cond");
    public static final SymbolicTerm MATCH  = Term.sym("org.aiddl.eval.match");
    public static final SymbolicTerm LET    = Term.sym("org.aiddl.eval.let");

    /**
     * org.aiddl.eval.symbolic
     */
    public static final SymbolicTerm SYM_CONCAT    = Term.sym("org.aiddl.eval.symbolic.concat");
    public static final SymbolicTerm SYM_SPLIT    = Term.sym("org.aiddl.eval.symbolic.split");

    /**
     * org.aiddl.eval.string
     */
    public static final SymbolicTerm STR_CONCAT    = Term.sym("org.aiddl.eval.string.concat");
    public static final SymbolicTerm STR_MOD_NAME    = Term.sym("org.aiddl.eval.string.module-folder");

    /**
     * org.aiddl.eval.collection
     */
    public static final SymbolicTerm IN = Term.sym("org.aiddl.eval.collection.in");
    public static final SymbolicTerm CONTAINS = Term.sym("org.aiddl.eval.collection.contains");
    public static final SymbolicTerm CONTAINS_ALL = Term.sym("org.aiddl.eval.collection.contains-all");
    public static final SymbolicTerm CONTAINS_ANY = Term.sym("org.aiddl.eval.collection.contains-any");
    public static final SymbolicTerm ADD_COL = Term.sym("org.aiddl.eval.collection.add-element");
    public static final SymbolicTerm ADD_COL_ALL = Term.sym("org.aiddl.eval.collection.add-all");
    public static final SymbolicTerm REM_COL = Term.sym("org.aiddl.eval.collection.remove");
    public static final SymbolicTerm REM_COL_ALL = Term.sym("org.aiddl.eval.collection.remove-all");
    public static final SymbolicTerm CONTAINS_MATCH = Term.sym("org.aiddl.eval.collection.contains-match");
    public static final SymbolicTerm SUM = Term.sym("org.aiddl.eval.collection.sum");
    /**
     * collection & tuple
     */
    public static final SymbolicTerm PUT_ALL = Term.sym("org.aiddl.eval.collection.put-all");
    public static final SymbolicTerm CONTAINS_KEY = Term.sym("org.aiddl.eval.contains-key");
    public static final SymbolicTerm IS_UNIQUE_MAP = Term.sym("org.aiddl.eval.is-unique-map");
    /**
     * org.aiddl.eval.set
     */
    public static final SymbolicTerm UNION	= Term.sym("org.aiddl.eval.set.union");
    /**
     * org.aiddl.eval.list
     */
    public static final SymbolicTerm CONCAT	= Term.sym("org.aiddl.eval.list.concat");
    /**
     * org.aiddl.eval.numeric
     */
    public static final SymbolicTerm ADD = Term.sym("org.aiddl.eval.numerical.add");
    public static final SymbolicTerm SUB = Term.sym("org.aiddl.eval.numerical.sub");
    public static final SymbolicTerm MULT = Term.sym("org.aiddl.eval.numerical.mult");
    public static final SymbolicTerm DIV = Term.sym("org.aiddl.eval.numerical.div");
    public static final SymbolicTerm MODULO = Term.sym("org.aiddl.eval.numerical.modulo");

    public static final SymbolicTerm GREATER_THAN = Term.sym("org.aiddl.eval.numerical.greater-than");
    public static final SymbolicTerm GREATER_THAN_EQ = Term.sym("org.aiddl.eval.numerical.greater-than-eq");
    public static final SymbolicTerm LESS_THAN = Term.sym("org.aiddl.eval.numerical.less-than");
    public static final SymbolicTerm LESS_THAN_EQ = Term.sym("org.aiddl.eval.numerical.less-than-eq");

    public static final SymbolicTerm IS_POSITIVE = Term.sym("org.aiddl.eval.numerical.is-negative");
    public static final SymbolicTerm IS_NEGATIVE = Term.sym("org.aiddl.eval.numerical.is-zero");
    public static final SymbolicTerm IS_ZERO = Term.sym("org.aiddl.eval.numerical.is-zero");
    public static final SymbolicTerm IS_NAN = Term.sym("org.aiddl.eval.numerical.is-nan");

    /**
     * org.aiddl.eval.logic
     */
    public static final SymbolicTerm AND = Term.sym("org.aiddl.eval.logic.and");
    public static final SymbolicTerm OR  = Term.sym("org.aiddl.eval.logic.or");
    public static final SymbolicTerm NOT = Term.sym("org.aiddl.eval.logic.not");
    public static final SymbolicTerm EXISTS = Term.sym("org.aiddl.eval.logic.exists");
    public static final SymbolicTerm FORALL = Term.sym("org.aiddl.eval.logic.forall");

    /**
     * org.aiddl.type
     */
    public static final SymbolicTerm TYPE_TERM = Term.sym("org.aiddl.type.term");
    public static final SymbolicTerm TYPE_NUMERICAL = Term.sym("org.aiddl.type.term.numerical");
    public static final SymbolicTerm TYPE_INTEGER = Term.sym("org.aiddl.type.term.numerical.integer");
    public static final SymbolicTerm TYPE_RATIONAL = Term.sym("org.aiddl.type.term.numerical.rational");
    public static final SymbolicTerm TYPE_REAL = Term.sym("org.aiddl.type.term.numerical.real");
    public static final SymbolicTerm TYPE_INF = Term.sym("org.aiddl.type.term.numerical.infinity");

    public static final SymbolicTerm TYPE_SYMBOLIC = Term.sym("org.aiddl.type.term.symbolic");
    public static final SymbolicTerm TYPE_BOOLEAN = Term.sym("org.aiddl.type.term.symbolic.boolean");
    public static final SymbolicTerm TYPE_STRING = Term.sym("org.aiddl.type.term.string");
    public static final SymbolicTerm TYPE_VARIABLE = Term.sym("org.aiddl.type.term.variable");
    public static final SymbolicTerm TYPE_REF = Term.sym("org.aiddl.type.term.reference");
    public static final SymbolicTerm TYPE_FREF = Term.sym("org.aiddl.type.term.fun-ref");

    public static final SymbolicTerm TYPE_COLLECTION = Term.sym("org.aiddl.type.term.collection");
    public static final SymbolicTerm TYPE_SET = Term.sym("org.aiddl.type.term.collection.set");
    public static final SymbolicTerm TYPE_LIST = Term.sym("org.aiddl.type.term.collection.list");
    public static final SymbolicTerm TYPE_TUPLE = Term.sym("org.aiddl.type.term.tuple");
    public static final SymbolicTerm TYPE_KEY_VALUE = Term.sym("org.aiddl.type.term.key-value");
}
