package org.aiddl.core.scala.function

import java.lang.reflect.Constructor
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.function.DefaultFunctionUri as D
import org.aiddl.core.scala.function.`type`.TypeCheckFunction
import org.aiddl.core.scala.function.higher_order.{FilterFunction, MapFunction, ReduceFunction}
import org.aiddl.core.scala.function.logic.{AndFunction, ExistsFunction, ForallFunction, OrFunction}
import org.aiddl.core.scala.function.misc.{CondFunction, DomainGenerationFunction, EvalAllRefsFunction, EvalRefFunction, IfFunction, LambdaFunctionEvaluator, LetFunction, Match, QuoteFunction, ZipFunction}
import org.aiddl.core.scala.function.numerical.{AdditionFunction, DivisionFunction, MultiplicationFunction, SubtractionFunction}
import org.aiddl.core.scala.function.scala.{FunctionFactoryLoader, FunctionLoader}
import org.aiddl.core.scala.tools.Logger
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.representation.Str
import org.aiddl.core.scala.representation.Integer
import org.aiddl.core.scala.representation.Rational
import org.aiddl.core.scala.representation.Real
import org.aiddl.core.scala.representation.InfPos
import org.aiddl.core.scala.representation.CollectionTerm
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.SetTerm
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.representation.EntRef
import org.aiddl.core.scala.representation.FunRef
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.InfNeg
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.TermImplicits.*

trait Function {
    def apply( x: Term ): Term
}

object Function {
    def loadDefaultFunctions( c: Container ) = {
        val eval = new Evaluator(c)

        c.addFunction(D.EVAL, eval)
        c.addFunction(D.EVAL_ALL_REFS, new EvalAllRefsFunction(eval))
        c.addFunction(D.EVAL_REF,  new EvalRefFunction(eval))
        c.addFunction(D.CORE_LANG, _ => Sym("scala"))

        c.addFunction(D.LOAD_FUNCTION, new FunctionLoader(c))
        c.addFunction(D.LOAD_FUNCTION_FACTORY, new FunctionFactoryLoader(c))
        
        c.addFunction(D.QUOTE, new QuoteFunction() )
        c.addFunction(D.CALL, x => x match { case Tuple(f, arg) => f(arg) case _ => x })
        c.addFunction(D.LAMBDA, new LambdaFunctionEvaluator(c))
        c.addFunction(D.MATCH, new Match(c))
        c.addFunction(D.SUBSTITUTE, x => x match { case Tuple(t, col: CollectionTerm) => t\Substitution.from(col) case _ => x })
        c.addFunction(D.MAP, new MapFunction(c))
        c.addFunction(D.FILTER, new FilterFunction(c))
        c.addFunction(D.REDUCE, new ReduceFunction(c))
        c.addFunction(D.LET, new LetFunction(c))

        c.addFunction(D.EQUALS, x => Bool(!x.isInstanceOf[Tuple] || x.asTup.tail.forall(e => x(0) == e)))
        c.addFunction(D.NOT_EQUALS, x => !Bool(!x.isInstanceOf[Tuple] || x.asTup.tail.forall(e => x(0) == e))) 
        c.addFunction(D.MATCHES, x => Bool(x.isInstanceOf[Tuple] && (x(0) unifiable x(1))))
        c.addFunction(D.SIZE, x => Integer(x.length))

        c.addFunction(D.SYM_CONCAT, x => x match { case Tuple(args @ _*) => args.tail.foldLeft(args.head)(_ + _) case _ => x } )
        c.addFunction(D.SYM_SPLIT, x => x match { case s @ Sym(_) => s.split case _ => x } )
                
        c.addFunction(D.STR_CONCAT, x => x.asTup.foldLeft(Str(""))((c, s) => c + s))

        c.addFunction(D.ADD, new AdditionFunction())
        c.addFunction(D.SUB, new SubtractionFunction())
        c.addFunction(D.MULT, new MultiplicationFunction())
        c.addFunction(D.DIV, new DivisionFunction())
        c.addFunction(D.MODULO, x => x match { case Tuple(n: Integer, d: Integer) => Num(n.x % d.x) case _ => x })
        
        c.addFunction(D.LESS_THAN, args => args match { case Tuple(x, y) => Bool(x < y) case _ => args } )
        c.addFunction(D.LESS_THAN_EQ, args => args match { case Tuple(x, y) => Bool(x <= y) case _ => args } )
        c.addFunction(D.GREATER_THAN, args => args match { case Tuple(x, y) => Bool(x > y) case _ => args } )
        c.addFunction(D.GREATER_THAN_EQ, args => args match { case Tuple(x, y) => Bool(x >= y) case _ => args } )

        c.addFunction(D.IS_NEGATIVE, args => args.isNeg);
        c.addFunction(D.IS_POSITIVE, args => args.isPos);
        c.addFunction(D.IS_ZERO, args => args.isZero);
        c.addFunction(D.IS_NAN, args => args.isNan);
        c.addFunction(D.IS_INF, args => args.isInf);
        c.addFunction(D.IS_INF_POS, args => args.isInfPos);
        c.addFunction(D.IS_INF_NEG, args => args.isInfNeg);

        c.addFunction(D.NOT, x => !x)
        c.addFunction(D.AND, new AndFunction(c))
        c.addFunction(D.OR, new OrFunction(c))
        c.addFunction(D.FORALL, new ForallFunction(c))
        c.addFunction(D.EXISTS, new ExistsFunction(c))

        c.addFunction(D.GET_KEY, x => x match { case Tuple(k, t) => t(k) case _ => ??? })

        c.addFunction(D.GET_IDX, x => x match { case Tuple(i, t) => t(i.asInt.x.intValue()) case _ => x })

        c.addFunction(D.IN, x => x match { case Tuple(e, c) => Bool(c.asCol.contains(e)) case _ => x })
        c.addFunction(D.CONTAINS, x => x match { case Tuple(c, e) => Bool(c.asCol.contains(e)) case _ => x })
        c.addFunction(D.CONTAINS_ALL, x => x match { case Tuple(c1, c2) => Bool(c1.asCol.containsAll(c2.asCol)) case _ => x })
        c.addFunction(D.CONTAINS_ANY, x => x match { case Tuple(c1, c2) => Bool(c1.asCol.containsAny(c2.asCol)) case _ => x })
        c.addFunction(D.CONTAINS_KEY, x => x match { case Tuple(c1, k) => Bool(c1.asCol.containsKey(k)) case _ => x })

        c.addFunction(D.REM_COL, x => x match { case Tuple(c, e) => c.asCol.remove(e) case _ => x })
        c.addFunction(D.REM_COL_ALL, x => x match { case Tuple(c1, c2) => c1.asCol.removeAll(c2.asCol) case _ => x })

        c.addFunction(D.ADD_COL, x => x match { case Tuple(c, e) => c.asCol.add(e) case _ => x })
        c.addFunction(D.ADD_COL_ALL, x => x match { case Tuple(c1, c2) => c1.asCol.addAll(c2.asCol) case _ => x })
        c.addFunction(D.PUT_ALL, x => x match { case Tuple(c1, c2) => c1.asCol.putAll(c2.asCol) case _ => x })

        c.addFunction(D.UNION, x => x match { case col: CollectionTerm => col.foldLeft(SetTerm())( (c, s) => c.addAll(s.asCol)) case _ => x })
        c.addFunction(D.CONCAT, x => x match { case col: CollectionTerm => col.foldLeft(ListTerm.empty)( (c, s) => c.addAll(s.asCol)) case _ => ??? })
        c.addFunction(D.FIRST, x => x.asCol.head)
        c.addFunction(D.LAST, x => x.asCol.last)

        c.addFunction(D.IF, new IfFunction(c))
        c.addFunction(D.COND, new CondFunction(c))
        c.addFunction(D.DOMAIN, new DomainGenerationFunction(c))
        c.addFunction(D.ZIP, new ZipFunction)

        c.addFunction(D.TYPE, new TypeCheckFunction(c))
        c.addFunction(D.TYPE_TERM, x => Bool(true))
        c.addFunction(D.TYPE_SYMBOLIC, x => Bool(x.isInstanceOf[Sym]))
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







