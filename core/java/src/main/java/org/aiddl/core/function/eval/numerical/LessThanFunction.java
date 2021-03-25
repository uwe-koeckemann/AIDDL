package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class LessThanFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool(((NumericalTerm) x.get(0)).lessThan((NumericalTerm) x.get(1)));
	}

}
