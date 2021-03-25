package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class AdditionFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		NumericalTerm sum = Term.integer(0);
		for ( int i = 0 ; i < x.size() ; i++ ) {
			sum = sum.add(x.get(i).asNum());
		}
		return sum;
	}
}
