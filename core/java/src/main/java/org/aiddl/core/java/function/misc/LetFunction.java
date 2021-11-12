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
 * Evaluate (let { x1 : v1 ... } c) to c with all xi substituted by vi.
 * 
 * @author Uwe Koeckemann
 */
public class LetFunction implements LazyFunction, ConfigurableFunction {
	
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public LetFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		Substitution s = new Substitution();
		for ( Term kvp : x.get(0).asCollection() ) {
			s.add(kvp.getKey(), this.eval.apply(kvp.getValue()));
		}
		return eval.apply( x.get(1).substitute(s) );
	}
}
