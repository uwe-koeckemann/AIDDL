package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class LessOrEqualsFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool(((NumericalTerm) x.get(0)).lessThanEq((NumericalTerm) x.get(1)));
	}

}
