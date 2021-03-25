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
 * Evaluate (let { x1 : v1 ... } c) to c with all xi substituted by vi.
 * 
 * @author Uwe Köckemann
 */
public class LetFunction implements Function, ConfigurableFunction {
	
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
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
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
