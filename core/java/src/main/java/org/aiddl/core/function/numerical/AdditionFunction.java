package org.aiddl.core.function.numerical;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class AdditionFunction implements Function {

	@Override
	public Term apply(Term x) {
		NumericalTerm sum = Term.integer(0);
		for ( int i = 0 ; i < x.size() ; i++ ) {
			sum = sum.add(x.get(i).asNum());
		}
		return sum;
	}
}
