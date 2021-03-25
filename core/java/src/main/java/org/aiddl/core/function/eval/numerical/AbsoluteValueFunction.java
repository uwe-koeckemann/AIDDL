package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class AbsoluteValueFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		NumericalTerm v = x.asNum();
		if ( v.isNegative() ) {
			return v.mult(Term.integer(-1));
		} else {
			return v;
		}
	}
}
