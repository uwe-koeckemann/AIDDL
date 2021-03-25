package org.aiddl.core.function;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.eval.AtIndexFunction;
import org.aiddl.core.function.eval.AtKeyFunction;
import org.aiddl.core.function.eval.CallFunction;
import org.aiddl.core.function.eval.EqualsFunction;
import org.aiddl.core.function.eval.EvalAllRefsFunction;
import org.aiddl.core.function.eval.ConditionalFunction;
import org.aiddl.core.function.eval.CoreLang;
import org.aiddl.core.function.eval.DomainExpansionFunction;
import org.aiddl.core.function.eval.FilterFunction;
import org.aiddl.core.function.eval.LetFunction;
import org.aiddl.core.function.eval.MapFunction;
import org.aiddl.core.function.eval.MatchFunction;
import org.aiddl.core.function.eval.ReduceFunction;
import org.aiddl.core.function.eval.ReferenceEvalFunction;
import org.aiddl.core.function.eval.SignatureCheckFunction;
import org.aiddl.core.function.eval.TypeCheckFunction;
import org.aiddl.core.function.eval.ZipFunction;
import org.aiddl.core.function.eval.FirstFunction;
import org.aiddl.core.function.eval.GetMatchingEntriesFunction;
import org.aiddl.core.function.eval.IfFunction;
import org.aiddl.core.function.eval.KeyFunction;
import org.aiddl.core.function.eval.LambdaFunctionEvaluator;
import org.aiddl.core.function.eval.LastFunction;
import org.aiddl.core.function.eval.MatchesFunction;
import org.aiddl.core.function.eval.NotEqualsFunction;
import org.aiddl.core.function.eval.QuoteFunction;
import org.aiddl.core.function.eval.SizeOfFunction;
import org.aiddl.core.function.eval.SubstitutionFunction;
import org.aiddl.core.function.eval.ValueFunction;
import org.aiddl.core.function.eval.collection.AddAllToCollectionFunction;
import org.aiddl.core.function.eval.collection.AddToCollectionFunction;
import org.aiddl.core.function.eval.collection.ContainsAllFunction;
import org.aiddl.core.function.eval.collection.ContainsAnyFunction;
import org.aiddl.core.function.eval.collection.ContainsFunction;
import org.aiddl.core.function.eval.collection.ContainsKeyFunction;
import org.aiddl.core.function.eval.collection.ContainsMatchFunction;
import org.aiddl.core.function.eval.collection.InFunction;
import org.aiddl.core.function.eval.collection.IsUniqueMapFunction;
import org.aiddl.core.function.eval.collection.RemoveAllFromCollectionFunction;
import org.aiddl.core.function.eval.collection.RemoveFromCollectionFunction;
import org.aiddl.core.function.eval.collection.SumFunction;
import org.aiddl.core.function.eval.list.ConcatFunction;
import org.aiddl.core.function.eval.logic.EvalAnd;
import org.aiddl.core.function.eval.logic.EvalExists;
import org.aiddl.core.function.eval.logic.EvalForall;
import org.aiddl.core.function.eval.logic.EvalOr;
import org.aiddl.core.function.eval.logic.NotFunction;
import org.aiddl.core.function.eval.math.CosineFunction;
import org.aiddl.core.function.eval.math.ExptFunction;
import org.aiddl.core.function.eval.math.SineFunction;
import org.aiddl.core.function.eval.numerical.AbsoluteValueFunction;
import org.aiddl.core.function.eval.numerical.AdditionFunction;
import org.aiddl.core.function.eval.numerical.DivideFunction;
import org.aiddl.core.function.eval.numerical.GreaterOrEqualsFunction;
import org.aiddl.core.function.eval.numerical.GreaterThanFunction;
import org.aiddl.core.function.eval.numerical.LessOrEqualsFunction;
import org.aiddl.core.function.eval.numerical.LessThanFunction;
import org.aiddl.core.function.eval.numerical.ModuloFunction;
import org.aiddl.core.function.eval.numerical.MultiplyFunction;
import org.aiddl.core.function.eval.numerical.SubtractFunction;
import org.aiddl.core.function.eval.random.NormalDistributionSampler;
import org.aiddl.core.function.eval.random.UniformElementSampler;
import org.aiddl.core.function.eval.random.UniformIntegerSampler;
import org.aiddl.core.function.eval.random.UniformRealSampler;
import org.aiddl.core.function.eval.set.UnionFunction;
import org.aiddl.core.function.eval.string.ModuleFolder;
import org.aiddl.core.function.eval.string.StringConcat;
import org.aiddl.core.function.eval.symbolic.SymbolicConcatFunction;
import org.aiddl.core.function.eval.symbolic.SymbolicSplitFunction;
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
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.service.RequestHandler;

/**
 * Create function registry containing all default functions. 
 * @author Uwe Köckemann
 *
 */
public class DefaultFunctions {
	
	public static final SymbolicTerm EVAL = Term.sym("org.aiddl.eval");
	public static final SymbolicTerm EVAL_REF = Term.sym("org.aiddl.eval.eval-ref");
	public static final SymbolicTerm EVAL_ALL_REFS = Term.sym("org.aiddl.eval.eval-all-refs");
	
	public static final SymbolicTerm LOAD_JAVA = Term.sym("org.aiddl.eval.load-function");
	public static final SymbolicTerm LOAD_JAVA_FACT = Term.sym("org.aiddl.eval.load-function-factory");
	
	public static final SymbolicTerm CORE_LANG = Term.sym("org.aiddl.eval.core-lang");
	
	public static final SymbolicTerm CALL_REQUEST = Term.sym("org.aiddl.eval.call-request");
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
	
	public static final SymbolicTerm ABS = Term.sym("org.aiddl.eval.numerical.absolute");	
	public static final SymbolicTerm POW = Term.sym("org.aiddl.eval.numerical.power");
	public static final SymbolicTerm SIN = Term.sym("org.aiddl.eval.numerical.sine");
	public static final SymbolicTerm COS = Term.sym("org.aiddl.eval.numerical.cosine");
	
	
	/**
	 * org.aiddl.eval.random
	 */
	public static final SymbolicTerm RAND_UNIFORM_REAL = Term.sym("org.aiddl.eval.random.uniform.real");
	public static final SymbolicTerm RAND_UNIFORM_INT = Term.sym("org.aiddl.eval.random.uniform.integer");	
	public static final SymbolicTerm RAND_UNIFORM_ELEMENT = Term.sym("org.aiddl.eval.random.uniform.element");
	public static final SymbolicTerm RAND_NORMAL = Term.sym("org.aiddl.eval.random.normal");
	
	/**
	 * org.aiddl.eval.logic
	 */
	public static final SymbolicTerm AND = Term.sym("org.aiddl.eval.logic.and");
	public static final SymbolicTerm OR  = Term.sym("org.aiddl.eval.logic.or");
	public static final SymbolicTerm NOT = Term.sym("org.aiddl.eval.logic.not");
	public static final SymbolicTerm EXISTS = Term.sym("org.aiddl.eval.logic.exists");
	public static final SymbolicTerm FORALL = Term.sym("org.aiddl.eval.logic.forall");
	
	public static final SymbolicTerm TYPE_TERM = Term.sym("org.aiddl.term");
	public static final SymbolicTerm TYPE_NUMERICAL = Term.sym("org.aiddl.term.numerical");
	public static final SymbolicTerm TYPE_INTEGER = Term.sym("org.aiddl.term.numerical.integer");
	public static final SymbolicTerm TYPE_RATIONAL = Term.sym("org.aiddl.term.numerical.rational");
	public static final SymbolicTerm TYPE_REAL = Term.sym("org.aiddl.term.numerical.real");
	public static final SymbolicTerm TYPE_INF = Term.sym("org.aiddl.term.numerical.infinity");
	
	public static final SymbolicTerm TYPE_SYMBOLIC = Term.sym("org.aiddl.term.symbolic");
	public static final SymbolicTerm TYPE_BOOLEAN = Term.sym("org.aiddl.term.symbolic.boolean");
	public static final SymbolicTerm TYPE_STRING = Term.sym("org.aiddl.term.string");
	public static final SymbolicTerm TYPE_VARIABLE = Term.sym("org.aiddl.term.variable");
	public static final SymbolicTerm TYPE_REF = Term.sym("org.aiddl.term.reference");
	public static final SymbolicTerm TYPE_FREF = Term.sym("org.aiddl.term.fun-ref");
	
	public static final SymbolicTerm TYPE_COLLECTION = Term.sym("org.aiddl.term.collection");
	public static final SymbolicTerm TYPE_SET = Term.sym("org.aiddl.term.collection.set");
	public static final SymbolicTerm TYPE_LIST = Term.sym("org.aiddl.term.collection.list");
	public static final SymbolicTerm TYPE_TUPLE = Term.sym("org.aiddl.term.tuple");
	public static final SymbolicTerm TYPE_KEY_VALUE = Term.sym("org.aiddl.term.key-value");

//	public static Evaluator DefaultEvaluator;
//	public static FunctionRegistry Registry;
	
	public static FunctionRegistry createDefaultRegistry( Container C ) {
		FunctionRegistry fReg = new FunctionRegistry();		
		
		Evaluator eval = new Evaluator( fReg, C );

		fReg.addFunction( EVAL, eval );
		fReg.addFunction( EVAL_REF, new ReferenceEvalFunction( eval ) );
		fReg.addFunction( EVAL_ALL_REFS, new EvalAllRefsFunction(eval) );
		
		fReg.addFunction( CALL_REQUEST, new RequestCaller(C, new RequestHandler(fReg)));
		fReg.addFunction( INIT_FUNCTION, new FunctionInitializer(fReg));
		fReg.addFunction( CONFIG_FUNCTION, new FunctionConfigurator(fReg));
		
		fReg.addFunction( LOAD_JAVA, new JavaFunctionLoader(fReg));
		fReg.addFunction( LOAD_JAVA_FACT, new JavaFunctionFactoryLoader(fReg) );
		
		fReg.addFunction( CORE_LANG, new CoreLang() );
		
		fReg.addFunction( CALL, new CallFunction());	
		fReg.addFunction( LAMBDA, new LambdaFunctionEvaluator(fReg));
				
		fReg.addFunction( FORALL, new EvalForall( eval ));	
		fReg.addFunction( EXISTS, new EvalExists( eval ));	
		fReg.addFunction( LET, new LetFunction( eval ));
		fReg.addFunction( TYPE, new TypeCheckFunction( fReg ));
		fReg.addFunction( SIGNATURE, new SignatureCheckFunction( eval ));
		fReg.addFunction( ZIP, new ZipFunction( eval ));
		fReg.addFunction( MAP, new MapFunction( eval ));
		fReg.addFunction( FILTER, new FilterFunction( eval ));
		fReg.addFunction( REDUCE, new ReduceFunction( eval ));
		fReg.addFunction( MATCH, new MatchFunction( eval ));
		fReg.addFunction( IF, new IfFunction( eval ));
		fReg.addFunction( COND, new ConditionalFunction( eval ));
		fReg.addFunction( AND, new EvalAnd( eval ) );
		fReg.addFunction( OR, new EvalOr( eval ) );
				
		fReg.addFunction( DOMAIN, new DomainExpansionFunction( eval ) );
		
		
		fReg.addFunction( QUOTE, new QuoteFunction() );
		
		fReg.addFunction( NOT, new NotFunction() );
		
		fReg.addFunction( IN, new InFunction() );
		fReg.addFunction( CONTAINS,	new ContainsFunction() );
		fReg.addFunction( CONTAINS_ALL, new ContainsAllFunction() );
		fReg.addFunction( CONTAINS_ANY, new ContainsAnyFunction() );
		fReg.addFunction( CONTAINS_MATCH, new ContainsMatchFunction() );		
		fReg.addFunction( CONTAINS_KEY, new ContainsKeyFunction() );
		fReg.addFunction( IS_UNIQUE_MAP, new IsUniqueMapFunction() );
		
		fReg.addFunction( ADD_COL, new AddToCollectionFunction() );
		fReg.addFunction( ADD_COL_ALL, new AddAllToCollectionFunction() );
		
		fReg.addFunction( REM_COL, new RemoveFromCollectionFunction() );
		fReg.addFunction( REM_COL_ALL, new RemoveAllFromCollectionFunction() );
		
		fReg.addFunction( UNION, new UnionFunction() );
		fReg.addFunction( CONCAT, new ConcatFunction() );
		
		fReg.addFunction( SYM_CONCAT, new SymbolicConcatFunction() );
		fReg.addFunction( SYM_SPLIT, new SymbolicSplitFunction() );
		
		fReg.addFunction( MATCHES, new MatchesFunction() );
		fReg.addFunction( EQUALS, new EqualsFunction() );
		fReg.addFunction( NOT_EQUALS, new NotEqualsFunction() );
		fReg.addFunction( SUBSTITUTE, new SubstitutionFunction() );
		fReg.addFunction( GET_MATCHING_ENTRIES, new GetMatchingEntriesFunction(C) );
		
		fReg.addFunction( FIRST, new FirstFunction() );
		fReg.addFunction( LAST, new LastFunction() );
		fReg.addFunction( KEY, new KeyFunction() );
		fReg.addFunction( VALUE, new ValueFunction() );
		fReg.addFunction( GET_IDX, new AtIndexFunction() );
		fReg.addFunction( GET_KEY, new AtKeyFunction() );
		fReg.addFunction( SIZE, new SizeOfFunction() );
		fReg.addFunction( SUM, new SumFunction() );
		
		/**
		 * NUMERICAL
		 */
		fReg.addFunction( ADD, new AdditionFunction());
		fReg.addFunction( MULT, new MultiplyFunction() );
		fReg.addFunction( SUB, new SubtractFunction());
		fReg.addFunction( DIV, new DivideFunction() );
		fReg.addFunction( MODULO, new ModuloFunction() );
		fReg.addFunction( GREATER_THAN, new GreaterThanFunction() );
		fReg.addFunction( GREATER_THAN_EQ, new GreaterOrEqualsFunction() );
		fReg.addFunction( LESS_THAN, new LessThanFunction() );
		fReg.addFunction( LESS_THAN_EQ, new LessOrEqualsFunction() );

//		fReg.addFunction( ABS, new AbsoluteValueFunction() );
//		fReg.addFunction( POW, new ExptFunction() );
//		fReg.addFunction( SIN, new SineFunction() );
//		fReg.addFunction( COS, new CosineFunction() );
//		
//		/**
//		 * RANDOM
//		 */
//		fReg.addFunction( RAND_UNIFORM_REAL, new UniformRealSampler());
//		fReg.addFunction( RAND_UNIFORM_INT, new UniformIntegerSampler());
//		fReg.addFunction( RAND_UNIFORM_ELEMENT, new UniformElementSampler());
//		fReg.addFunction( RAND_NORMAL, new NormalDistributionSampler());
		
		/**
		 * STRING
		 */
		fReg.addFunction( STR_CONCAT, new StringConcat() );
		fReg.addFunction( STR_MOD_NAME, new ModuleFolder() );
		
		
		/**
		 * BASIC TYPES
		 */
		fReg.addFunction( TYPE_TERM, new TermTypeFunction());
		fReg.addFunction( TYPE_NUMERICAL, new NumericalTypeFunction());
		fReg.addFunction( TYPE_INTEGER, new IntegerTypeFunction());
		fReg.addFunction( TYPE_RATIONAL, new RationalTypeFunction());
		fReg.addFunction( TYPE_REAL, new RealTypeFunction());
		fReg.addFunction( TYPE_INF, new InfinityTypeFunction());
		
		fReg.addFunction( TYPE_SYMBOLIC, new SymbolicTypeFunction());
		fReg.addFunction( TYPE_BOOLEAN, new BooleanTypeFunction());
		
		fReg.addFunction( TYPE_STRING, new StringTypeFunction());
		fReg.addFunction( TYPE_VARIABLE, new VariableTypeFunction());
		fReg.addFunction( TYPE_REF, new ReferenceTypeFunction());
		fReg.addFunction( TYPE_FREF, new FunctionReferenceTypeFunction());
		
		fReg.addFunction( TYPE_COLLECTION, new CollectionTypeFunction());
		fReg.addFunction( TYPE_LIST, new ListTypeFunction());
		fReg.addFunction( TYPE_SET, new SetTypeFunction());
		fReg.addFunction( TYPE_TUPLE, new TupleTypeFunction());
		fReg.addFunction( TYPE_KEY_VALUE, new KeyValueTypeFunction());
		
		fReg.loadContainerDefintions(C);
		fReg.loadContainerInterfaces(C);
		
		return fReg;
	}
}
