package org.aiddl.core.java.function.numerical;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class GreaterOrEqualsFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(((NumericalTerm) x.get(0)).greaterThanEq((NumericalTerm) x.get(1)));
	}

}
