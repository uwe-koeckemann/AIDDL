package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class IsNanFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool((x instanceof NumericalTerm) && x.asNum().isNaN());
	}

}
