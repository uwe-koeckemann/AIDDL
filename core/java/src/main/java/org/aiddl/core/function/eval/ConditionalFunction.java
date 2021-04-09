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
 * Evaluator for conditional expressions. A conditional expression 
 * has the form (cond x1 : v1 ... xn : vn) and evaluates to vi
 * if xi evaluates to true.
 *   
 * @author Uwe Koeckemann
 */
public class ConditionalFunction implements Function, ConfigurableFunction {
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public ConditionalFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		for ( int i = 0 ; i < x.size() ; i++ ) {
			if ( eval.apply(x.get(i).getKey()).getBooleanValue() ) {
				return eval.apply(x.get(i).getValue());
			}
		}
		throw new IllegalArgumentException("Conditional: " + x + " does not cover all cases.\nMake sure the existing conditions cover all cases, or add a catch all case (true) : <default-result> at the end to avoid this.");
	}
}
