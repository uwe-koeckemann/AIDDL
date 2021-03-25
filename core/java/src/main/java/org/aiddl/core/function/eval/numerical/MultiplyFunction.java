package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class MultiplyFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		NumericalTerm prod = Term.integer(1);
		for ( int i = 0 ; i < x.size() ; i++ ) {
			prod = prod.mult(x.get(i).asNum());
		}
		return prod;
	}
}
