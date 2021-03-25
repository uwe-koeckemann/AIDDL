package org.aiddl.core.function.eval;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

/**
 * Evaluate a term and its result.
 * 
 * @author Uwe Köckemann
 */
public class ReferenceEvalFunction implements Function, ConfigurableFunction {
	
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public ReferenceEvalFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		return eval.apply(eval.apply(x));
	}
}
