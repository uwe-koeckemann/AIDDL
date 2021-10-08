package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class NumericalTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof NumericalTerm);
	}

}
