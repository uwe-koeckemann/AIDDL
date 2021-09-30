package org.aiddl.core.function;

import org.aiddl.core.container.Container;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.java.JavaFunctionFactoryLoader;
import org.aiddl.core.function.java.JavaFunctionLoader;
import org.aiddl.core.function.misc.AtIndexFunction;
import org.aiddl.core.function.misc.AtKeyFunction;
import org.aiddl.core.function.higher_order.CallFunction;
import org.aiddl.core.function.misc.EqualsFunction;
import org.aiddl.core.function.misc.EvalAllRefsFunction;
import org.aiddl.core.function.misc.ConditionalFunction;
import org.aiddl.core.function.misc.CoreLang;
import org.aiddl.core.function.misc.DomainExpansionFunction;
import org.aiddl.core.function.higher_order.FilterFunction;
import org.aiddl.core.function.misc.LetFunction;
import org.aiddl.core.function.higher_order.MapFunction;
import org.aiddl.core.function.misc.MatchFunction;
import org.aiddl.core.function.higher_order.ReduceFunction;
import org.aiddl.core.function.misc.ReferenceEvalFunction;
import org.aiddl.core.function.type.SignatureCheckFunction;
import org.aiddl.core.function.type.TypeCheckFunction;
import org.aiddl.core.function.misc.ZipFunction;
import org.aiddl.core.function.collection.FirstFunction;
import org.aiddl.core.function.misc.GetMatchingEntriesFunction;
import org.aiddl.core.function.misc.IfFunction;
import org.aiddl.core.function.misc.KeyFunction;
import org.aiddl.core.function.misc.LambdaFunctionEvaluator;
import org.aiddl.core.function.collection.LastFunction;
import org.aiddl.core.function.misc.MatchesFunction;
import org.aiddl.core.function.misc.NotEqualsFunction;
import org.aiddl.core.function.collection.PutAllFunction;
import org.aiddl.core.function.misc.QuoteFunction;
import org.aiddl.core.function.misc.SizeOfFunction;
import org.aiddl.core.function.misc.SubstitutionFunction;
import org.aiddl.core.function.misc.ValueFunction;
import org.aiddl.core.function.collection.AddAllToCollectionFunction;
import org.aiddl.core.function.collection.AddToCollectionFunction;
import org.aiddl.core.function.collection.ContainsAllFunction;
import org.aiddl.core.function.collection.ContainsAnyFunction;
import org.aiddl.core.function.collection.ContainsFunction;
import org.aiddl.core.function.collection.ContainsKeyFunction;
import org.aiddl.core.function.collection.ContainsMatchFunction;
import org.aiddl.core.function.collection.InFunction;
import org.aiddl.core.function.collection.IsUniqueMapFunction;
import org.aiddl.core.function.collection.RemoveAllFromCollectionFunction;
import org.aiddl.core.function.collection.RemoveFromCollectionFunction;
import org.aiddl.core.function.collection.SumFunction;
import org.aiddl.core.function.collection.ConcatFunction;
import org.aiddl.core.function.logic.EvalAnd;
import org.aiddl.core.function.logic.EvalExists;
import org.aiddl.core.function.logic.EvalForall;
import org.aiddl.core.function.logic.EvalOr;
import org.aiddl.core.function.logic.NotFunction;
import org.aiddl.core.function.numerical.AdditionFunction;
import org.aiddl.core.function.numerical.DivideFunction;
import org.aiddl.core.function.numerical.GreaterOrEqualsFunction;
import org.aiddl.core.function.numerical.GreaterThanFunction;
import org.aiddl.core.function.numerical.LessOrEqualsFunction;
import org.aiddl.core.function.numerical.LessThanFunction;
import org.aiddl.core.function.numerical.ModuloFunction;
import org.aiddl.core.function.numerical.MultiplyFunction;
import org.aiddl.core.function.numerical.SubtractFunction;
import org.aiddl.core.function.collection.UnionFunction;
import org.aiddl.core.function.string.ModuleFolder;
import org.aiddl.core.function.string.StringConcat;
import org.aiddl.core.function.symbolic.SymbolicConcatFunction;
import org.aiddl.core.function.symbolic.SymbolicSplitFunction;
import org.aiddl.core.function.type.BooleanTypeFunction;
import org.aiddl.core.function.type.CollectionTypeFunction;
import org.aiddl.core.function.type.FunctionReferenceTypeFunction;
import org.aiddl.core.function.type.InfinityTypeFunction;
import org.aiddl.core.function.type.IntegerTypeFunction;
import org.aiddl.core.function.type.KeyValueTypeFunction;
import org.aiddl.core.function.type.ListTypeFunction;
import org.aiddl.core.function.type.NumericalTypeFunction;
import org.aiddl.core.function.type.RationalTypeFunction;
import org.aiddl.core.function.type.RealTypeFunction;
import org.aiddl.core.function.type.ReferenceTypeFunction;
import org.aiddl.core.function.type.SetTypeFunction;
import org.aiddl.core.function.type.StringTypeFunction;
import org.aiddl.core.function.type.SymbolicTypeFunction;
import org.aiddl.core.function.type.TermTypeFunction;
import org.aiddl.core.function.type.TupleTypeFunction;
import org.aiddl.core.function.type.VariableTypeFunction;
import org.aiddl.core.representation.Term;

/**
 * Create function registry containing all default functions. 
 * @author Uwe Koeckemann
 *
 */
public class DefaultFunctions {
	public static FunctionRegistry createDefaultRegistry( Container C ) {
		FunctionRegistry fReg = new FunctionRegistry();		
		
		Evaluator eval = new Evaluator( fReg, C );

		fReg.addFunction( Uri.EVAL, eval );
		fReg.addFunction( Uri.EVAL_REF, new ReferenceEvalFunction( eval ) );
		fReg.addFunction( Uri.EVAL_ALL_REFS, new EvalAllRefsFunction(eval) );
		
		fReg.addFunction( Uri.INIT_FUNCTION, new FunctionInitializer(fReg));
		fReg.addFunction( Uri.CONFIG_FUNCTION, new FunctionConfigurator(fReg));
		
		fReg.addFunction( Uri.LOAD_JAVA, new JavaFunctionLoader(fReg));
		fReg.addFunction( Uri.LOAD_JAVA_FACT, new JavaFunctionFactoryLoader(fReg) );
		
		fReg.addFunction( Uri.CORE_LANG, new CoreLang() );
		
		fReg.addFunction( Uri.CALL, new CallFunction());	
		fReg.addFunction( Uri.LAMBDA, new LambdaFunctionEvaluator(fReg));
				
		fReg.addFunction( Uri.FORALL, new EvalForall( eval ));	
		fReg.addFunction( Uri.EXISTS, new EvalExists( eval ));	
		fReg.addFunction( Uri.LET, new LetFunction( eval ));
		fReg.addFunction( Uri.TYPE, new TypeCheckFunction( fReg ));
		fReg.addFunction( Uri.SIGNATURE, new SignatureCheckFunction( eval ));
		fReg.addFunction( Uri.ZIP, new ZipFunction( eval ));
		fReg.addFunction( Uri.MAP, new MapFunction( eval ));
		fReg.addFunction( Uri.FILTER, new FilterFunction( eval ));
		fReg.addFunction( Uri.REDUCE, new ReduceFunction( eval ));
		fReg.addFunction( Uri.MATCH, new MatchFunction( eval ));
		fReg.addFunction( Uri.IF, new IfFunction( eval ));
		fReg.addFunction( Uri.COND, new ConditionalFunction( eval ));
		fReg.addFunction( Uri.AND, new EvalAnd( eval ) );
		fReg.addFunction( Uri.OR, new EvalOr( eval ) );
				
		fReg.addFunction( Uri.DOMAIN, new DomainExpansionFunction( eval ) );
		
		
		fReg.addFunction( Uri.QUOTE, new QuoteFunction() );
		
		fReg.addFunction( Uri.NOT, new NotFunction() );
		
		fReg.addFunction( Uri.IN, new InFunction() );
		fReg.addFunction( Uri.CONTAINS,	new ContainsFunction() );
		fReg.addFunction( Uri.CONTAINS_ALL, new ContainsAllFunction() );
		fReg.addFunction( Uri.CONTAINS_ANY, new ContainsAnyFunction() );
		fReg.addFunction( Uri.CONTAINS_MATCH, new ContainsMatchFunction() );		
		fReg.addFunction( Uri.CONTAINS_KEY, new ContainsKeyFunction() );
		fReg.addFunction( Uri.IS_UNIQUE_MAP, new IsUniqueMapFunction() );
		
		fReg.addFunction( Uri.PUT_ALL, new PutAllFunction() );
		
		fReg.addFunction( Uri.ADD_COL, new AddToCollectionFunction() );
		fReg.addFunction( Uri.ADD_COL_ALL, new AddAllToCollectionFunction() );
		
		fReg.addFunction( Uri.REM_COL, new RemoveFromCollectionFunction() );
		fReg.addFunction( Uri.REM_COL_ALL, new RemoveAllFromCollectionFunction() );
		
		fReg.addFunction( Uri.UNION, new UnionFunction() );
		fReg.addFunction( Uri.CONCAT, new ConcatFunction() );

		fReg.addFunction( Uri.SYM_CONCAT, new SymbolicConcatFunction() );
		fReg.addFunction( Uri.SYM_SPLIT, new SymbolicSplitFunction() );
		
		fReg.addFunction( Uri.MATCHES, new MatchesFunction() );
		fReg.addFunction( Uri.EQUALS, new EqualsFunction() );
		fReg.addFunction( Uri.NOT_EQUALS, new NotEqualsFunction() );
		fReg.addFunction( Uri.SUBSTITUTE, new SubstitutionFunction() );
		fReg.addFunction( Uri.GET_MATCHING_ENTRIES, new GetMatchingEntriesFunction(C) );
		
		fReg.addFunction( Uri.FIRST, new FirstFunction() );
		fReg.addFunction( Uri.LAST, new LastFunction() );
		fReg.addFunction( Uri.KEY, new KeyFunction() );
		fReg.addFunction( Uri.VALUE, new ValueFunction() );
		fReg.addFunction( Uri.GET_IDX, new AtIndexFunction() );
		fReg.addFunction( Uri.GET_KEY, new AtKeyFunction() );
		fReg.addFunction( Uri.SIZE, new SizeOfFunction() );
		fReg.addFunction( Uri.SUM, new SumFunction() );
		
		/**
		 * NUMERICAL
		 */
		fReg.addFunction( Uri.ADD, new AdditionFunction());
		fReg.addFunction( Uri.MULT, new MultiplyFunction() );
		fReg.addFunction( Uri.SUB, new SubtractFunction());
		fReg.addFunction( Uri.DIV, new DivideFunction() );
		fReg.addFunction( Uri.MODULO, new ModuloFunction() );
		fReg.addFunction( Uri.GREATER_THAN, new GreaterThanFunction() );
		fReg.addFunction( Uri.GREATER_THAN_EQ, new GreaterOrEqualsFunction() );
		fReg.addFunction( Uri.LESS_THAN, new LessThanFunction() );
		fReg.addFunction( Uri.LESS_THAN_EQ, new LessOrEqualsFunction() );
		fReg.addFunction( Uri.IS_POSITIVE, (x) -> Term.bool(x.asNum().isPositive()) );
		fReg.addFunction( Uri.IS_ZERO, (x) -> Term.bool(x.asNum().isZero()) );
		fReg.addFunction( Uri.IS_NEGATIVE, (x) -> Term.bool(x.asNum().isNegative()) );
		fReg.addFunction( Uri.IS_NAN, (x) -> Term.bool(x.asNum().isNaN()) );

		/**
		 * STRING
		 */
		fReg.addFunction( Uri.STR_CONCAT, new StringConcat() );
		fReg.addFunction( Uri.STR_MOD_NAME, new ModuleFolder() );

		/**
		 * BASIC TYPES
		 */
		fReg.addFunction( Uri.TYPE_TERM, new TermTypeFunction());
		fReg.addFunction( Uri.TYPE_NUMERICAL, new NumericalTypeFunction());
		fReg.addFunction( Uri.TYPE_INTEGER, new IntegerTypeFunction());
		fReg.addFunction( Uri.TYPE_RATIONAL, new RationalTypeFunction());
		fReg.addFunction( Uri.TYPE_REAL, new RealTypeFunction());
		fReg.addFunction( Uri.TYPE_INF, new InfinityTypeFunction());
		
		fReg.addFunction( Uri.TYPE_SYMBOLIC, new SymbolicTypeFunction());
		fReg.addFunction( Uri.TYPE_BOOLEAN, new BooleanTypeFunction());
		
		fReg.addFunction( Uri.TYPE_STRING, new StringTypeFunction());
		fReg.addFunction( Uri.TYPE_VARIABLE, new VariableTypeFunction());
		fReg.addFunction( Uri.TYPE_REF, new ReferenceTypeFunction());
		fReg.addFunction( Uri.TYPE_FREF, new FunctionReferenceTypeFunction());
		
		fReg.addFunction( Uri.TYPE_COLLECTION, new CollectionTypeFunction());
		fReg.addFunction( Uri.TYPE_LIST, new ListTypeFunction());
		fReg.addFunction( Uri.TYPE_SET, new SetTypeFunction());
		fReg.addFunction( Uri.TYPE_TUPLE, new TupleTypeFunction());
		fReg.addFunction( Uri.TYPE_KEY_VALUE, new KeyValueTypeFunction());
		
		fReg.loadContainerDefintions(C);
		fReg.loadContainerInterfaces(C);
		
		return fReg;
	}
}
