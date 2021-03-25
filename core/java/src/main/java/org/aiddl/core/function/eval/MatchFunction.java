package org.aiddl.core.function.eval;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

/**
 * Evaluate (match x y c) to c after applying a substitution that
 * makes x and y equal.
 * @author Uwe Köckemann
 *
 */
public class MatchFunction implements Function, ConfigurableFunction {
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
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
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
