package org.aiddl.core.function.numerical;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class LessOrEqualsFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(((NumericalTerm) x.get(0)).lessThanEq((NumericalTerm) x.get(1)));
	}

}
