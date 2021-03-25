package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

/**
 * Evaluate a term and its result.
 * 
 * @author Uwe Köckemann
 */
public class CallFunction implements Function {
		
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public CallFunction() {
	}
	
	@Override
	public Term apply(Term x) {
		Function f = x.get(0).asFunRef().getFunction();
		if ( f == null ) {
			throw new IllegalArgumentException("Function not registered: " + x.get(0));
		}
		return f.apply(x.get(1));
	}
}
