package org.aiddl.core.java.function.misc;

import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluate a term and its result.
 * 
 * @author Uwe Koeckemann
 */
public class EvalAllRefsFunction implements LazyFunction, ConfigurableFunction {
	
	Evaluator eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public EvalAllRefsFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = (Evaluator)fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		this.eval.setEvalAllReferences(true);		
		Term r = this.eval.apply(x);
		this.eval.setEvalAllReferences(false);		
		return r;
	}
}
