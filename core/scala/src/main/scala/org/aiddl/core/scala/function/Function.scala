package org.aiddl.core.scala.function

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.DefaultFunctionUri as D
import org.aiddl.core.scala.function.`type`.TypeCheckFunction
import org.aiddl.core.scala.function.higher_order.{FilterFunction, MapFunction, ReduceFunction}
import org.aiddl.core.scala.function.logic.{AndFunction, ExistsFunction, ForallFunction, OrFunction}
import org.aiddl.core.scala.function.misc.*
import org.aiddl.core.scala.function.numerical.{AdditionFunction, DivisionFunction, MultiplicationFunction, SubtractionFunction}
import org.aiddl.core.scala.function.scala.{FunctionFactoryLoader, FunctionLoader}
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, EntRef, FunRef, InfNeg, InfPos, Integer, KeyVal, ListTerm, Num, Rational, Real, SetTerm, Str, Substitution, Sym, Term, Tuple, Var}
import Term.{given_Conversion_Term_KeyVal, given_Conversion_Term_Sym}
import org.aiddl.core.scala.util.Logger

import java.lang.reflect.Constructor

/**
 * An AIDDL function takes a term as an argument and returns a term.
 * This trait is used to store arbitrary functions in function references.
 */
trait Function {
    /**
     * Abstraction of any computation done on AIDDL terms.
     * Functions that require multiple inputs can use tuple terms.
     * @param x input term
     * @return result of applying the function to <code>x</code>
     */
    def apply( x: Term ): Term
}

object Function {
    /**
     * Load default functions into container. This allows to evaluate terms in a Lisp-like way by using the evaluator.
     * The evaluator is just another function, but can be accessed directly from the container.
     * @param c container to load functions into
     */
    def loadDefaultFunctions( c: Container ): Unit = {
        val eval = new Evaluator(c)

        c.addFunction(D.EVAL, eval)
        c.addFunction(D.EVAL_ALL_REFS, new EvalAllRefsFunction(eval))
        c.addFunction(D.EVAL_REF,  new EvalRefFunction(eval))
        c.addFunction(D.CORE_LANG, _ => Sym("scala"))

        c.addFunction(D.LOAD_FUNCTION, new FunctionLoader(c))
        c.addFunction(D.LOAD_FUNCTION_FACTORY, new FunctionFactoryLoader(c))
        
        c.addFunction(D.QUOTE, new QuoteFunction() )
        c.addFunction(D.CALL, { case Tuple(f, arg) => f(arg) case x => x })
        c.addFunction(D.LAMBDA, new LambdaFunctionEvaluator(c))
        c.addFunction(D.MATCH, new Match(eval))
        c.addFunction(D.SUBSTITUTE, { case Tuple(t, col: CollectionTerm) => t \ Substitution.from(col) case x => x })
        c.addFunction(D.MAP, new MapFunction(eval))
        c.addFunction(D.FILTER, new FilterFunction(eval))
        c.addFunction(D.REDUCE, new ReduceFunction(eval))

        c.addFunction(D.LET, new LetFunction(eval))

        c.addFunction(D.EQUALS, x => Bool(!x.isInstanceOf[Tuple] || x.asTup.tail.forall(e => x(0) == e)))
        c.addFunction(D.NOT_EQUALS, x => !Bool(!x.isInstanceOf[Tuple] || x.asTup.tail.forall(e => x(0) == e))) 
        c.addFunction(D.MATCHES, x => Bool(x.isInstanceOf[Tuple] && (x(0) unifiable x(1))))
        c.addFunction(D.SIZE, x => Integer(x.length))

        c.addFunction(D.KEY, x => x.asKvp.key)
        c.addFunction(D.VALUE, x => x.asKvp.value)

        c.addFunction(D.SYM_CONCAT, { case Tuple(args@_*) => args.tail.foldLeft(args.head)(_.asSym + _.asSym) case x => x } )
        c.addFunction(D.SYM_SPLIT, { case s@Sym(_) => s.split case x => x } )
                
        c.addFunction(D.STR_CONCAT, x => x.asTup.foldLeft(Str(""))((c, s) => c + s))

        c.addFunction(D.ADD, new AdditionFunction())
        c.addFunction(D.SUB, new SubtractionFunction())
        c.addFunction(D.MULT, new MultiplicationFunction())
        c.addFunction(D.DIV, new DivisionFunction())
        c.addFunction(D.MODULO, { case Tuple(n: Integer, d: Integer) => Num(n.x % d.x) case x => x })
        
        c.addFunction(D.LESS_THAN, { case Tuple(x: Num, y: Num) => Bool(x < y) case args => args } )
        c.addFunction(D.LESS_THAN_EQ, { case Tuple(x: Num, y: Num) => Bool(x <= y) case args => args } )
        c.addFunction(D.GREATER_THAN, { case Tuple(x: Num, y: Num) => Bool(x > y) case args => args } )
        c.addFunction(D.GREATER_THAN_EQ, { case Tuple(x: Num, y: Num) => Bool(x >= y) case args => args } )

        c.addFunction(D.IS_NEGATIVE, args => args.asNum.isNeg)
        c.addFunction(D.IS_POSITIVE, args => args.asNum.isPos)
        c.addFunction(D.IS_ZERO, args => args.asNum.isZero)
        c.addFunction(D.IS_NAN, args => args.asNum.isNan)
        c.addFunction(D.IS_INF, args => args.asNum.isInf)
        c.addFunction(D.IS_INF_POS, args => args.asNum.isInfPos)
        c.addFunction(D.IS_INF_NEG, args => args.asNum.isInfNeg)

        c.addFunction(D.NOT, x => !x)
        c.addFunction(D.AND, new AndFunction(eval))
        c.addFunction(D.OR, new OrFunction(eval))
        c.addFunction(D.FORALL, new ForallFunction(eval))
        c.addFunction(D.EXISTS, new ExistsFunction(eval))

        c.addFunction(D.GET_KEY, {
            case Tuple(k, t) => t(k)
            case x => throw IllegalArgumentException(s"Unsupported argument: $x. " +
              s"${D.GET_KEY} requires arguments (k t) and will return t(k).") })

        c.addFunction(D.GET_IDX, { case Tuple(i, t) => t(i.asInt.x.intValue()) case x => x })

        c.addFunction(D.IN, { case Tuple(e, c) => Bool(c.asCol.contains(e)) case x => x })
        c.addFunction(D.CONTAINS, { case Tuple(c, e) => Bool(c.asCol.contains(e)) case x => x })
        c.addFunction(D.CONTAINS_ALL, { case Tuple(c1, c2) => Bool(c1.asCol.containsAll(c2.asCol)) case x => x })
        c.addFunction(D.CONTAINS_ANY, { case Tuple(c1, c2) => Bool(c1.asCol.containsAny(c2.asCol)) case x => x })
        c.addFunction(D.CONTAINS_KEY, { case Tuple(c1, k) => Bool(c1.asCol.containsKey(k)) case x => x })
        c.addFunction(D.IS_UNIQUE_MAP, x => x.asCol.forall( e => e.isInstanceOf[KeyVal] && !x.asCol.exists( e2 => (e2.asKvp.key == e.asKvp.key) && (e2.asKvp.value != e.asKvp.value) ) ))


        c.addFunction(D.REM_COL, { case Tuple(c, e) => c.asCol.remove(e) case x => x })
        c.addFunction(D.REM_COL_ALL, { case Tuple(c1, c2) => c1.asCol.removeAll(c2.asCol) case x => x })

        c.addFunction(D.ADD_COL, { case Tuple(c, e) => c.asCol.add(e) case x => x })
        c.addFunction(D.ADD_COL_ALL, { case Tuple(c1, c2) => c1.asCol.addAll(c2.asCol) case x => x })
        c.addFunction(D.PUT_ALL, { case Tuple(c1, c2) => c1.asCol.putAll(c2.asCol) case x => x })

        c.addFunction(D.UNION, { case col: CollectionTerm => col.foldLeft(SetTerm())((c, s) => c.addAll(s.asCol)) case x => x })
        c.addFunction(D.CONCAT, {
            case col: CollectionTerm => col.foldLeft(ListTerm.empty)((c, s) => c.addAll(s.asCol))
            case x => throw IllegalArgumentException(s"Unsupported argument: $x. " +
              s"${D.CONCAT} requires collection of collections. Example: [{a] [b] [c d]] and will return list [a b c d].") })
        c.addFunction(D.FIRST, x => x.asCol.head)
        c.addFunction(D.LAST, x => x.asCol.last)

        c.addFunction(D.IF, new IfFunction(eval))
        c.addFunction(D.COND, new CondFunction(eval))
        c.addFunction(D.DOMAIN, new DomainGenerationFunction(eval))
        c.addFunction(D.ZIP, new ZipFunction)

        c.addFunction(D.TYPE, new TypeCheckFunction(c))
        c.addFunction(D.TYPE_TERM, _ => Bool(true))
        c.addFunction(D.TYPE_SYMBOLIC, x => Bool(x.isInstanceOf[Sym] || x.isInstanceOf[Bool]))
        c.addFunction(D.TYPE_VARIABLE, x => Bool(x.isInstanceOf[Var]))
        c.addFunction(D.TYPE_STRING, x => Bool(x.isInstanceOf[Str]))
        c.addFunction(D.TYPE_NUMERICAL, x => Bool(x.isInstanceOf[Num]))
        c.addFunction(D.TYPE_INTEGER, x => Bool(x.isInstanceOf[Integer]))
        c.addFunction(D.TYPE_RATIONAL, x => Bool(x.isInstanceOf[Rational]))
        c.addFunction(D.TYPE_REAL, x => Bool(x.isInstanceOf[Real]))
        c.addFunction(D.TYPE_INF, x => Bool(x.isInstanceOf[InfPos] || x.isInstanceOf[InfNeg]))
        c.addFunction(D.TYPE_COLLECTION, x => Bool(x.isInstanceOf[CollectionTerm]))
        c.addFunction(D.TYPE_LIST, x => Bool(x.isInstanceOf[ListTerm]))
        c.addFunction(D.TYPE_SET, x => Bool(x.isInstanceOf[SetTerm]))
        c.addFunction(D.TYPE_TUPLE, x => Bool(x.isInstanceOf[Tuple]))
        c.addFunction(D.TYPE_KEY_VALUE, x => Bool(x.isInstanceOf[KeyVal]))
        c.addFunction(D.TYPE_EREF, x => Bool(x.isInstanceOf[EntRef]))
        c.addFunction(D.TYPE_FREF, x => Bool(x.isInstanceOf[FunRef]))
        c.addFunction(D.TYPE_BOOLEAN, x => Bool(x.isInstanceOf[Bool]))
    }
}







