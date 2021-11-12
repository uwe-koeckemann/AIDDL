package org.aiddl.core.java.function.numerical;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class AbsoluteValueFunction implements Function {

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
