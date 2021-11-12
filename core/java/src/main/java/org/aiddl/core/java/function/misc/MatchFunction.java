package org.aiddl.core.java.function.misc;

import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluate (match x y c) to c after applying a substitution that
 * makes x and y equal.
 * @author Uwe Koeckemann
 *
 */
public class MatchFunction implements LazyFunction, ConfigurableFunction {
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public MatchFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		Term from = eval.apply(x.get(0));
		Term to = eval.apply(x.get(1));
		Term matchConstraint = x.get(2); 	
		
		Substitution s = from.match(to);
		
		if ( s == null ) {
			return Term.bool(false);
		}	
		return eval.apply(matchConstraint.substitute(s));
	}

}
