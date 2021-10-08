package org.aiddl.core.function.logic;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.LazyFunction;
import org.aiddl.core.representation.Term;

/**
 * Evaluate (or x1 ...) to true if any xi is true.
 * @author Uwe Koeckemann
 */
public class EvalOr implements LazyFunction, ConfigurableFunction {

	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public EvalOr( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		if ( x.size() == 0 ) {
			return Term.bool(false);
		}	
		for ( int i = 0 ; i < x.size() ; i++ ) {
			if ( eval.apply(x.get(i)).getBooleanValue() ) {
				return Term.bool(true);
			}
		}
		return Term.bool(false);
	}

}
