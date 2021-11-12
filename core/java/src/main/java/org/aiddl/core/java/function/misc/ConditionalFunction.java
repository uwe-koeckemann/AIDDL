package org.aiddl.core.java.function.misc;

import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluator for conditional expressions. A conditional expression 
 * has the form (cond x1 : v1 ... xn : vn) and evaluates to vi
 * if xi evaluates to true.
 *   
 * @author Uwe Koeckemann
 */
public class ConditionalFunction implements LazyFunction, ConfigurableFunction {
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
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
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
