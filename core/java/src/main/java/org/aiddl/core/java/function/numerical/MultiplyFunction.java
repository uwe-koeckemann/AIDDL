package org.aiddl.core.java.function.numerical;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class MultiplyFunction implements Function {

	@Override
	public Term apply(Term x) {
		NumericalTerm prod = Term.integer(1);
		for ( int i = 0 ; i < x.size() ; i++ ) {
			prod = prod.mult(x.get(i).asNum());
		}
		return prod;
	}
}
