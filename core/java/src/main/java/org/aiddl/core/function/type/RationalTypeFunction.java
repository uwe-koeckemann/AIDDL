package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.RationalTerm;
import org.aiddl.core.representation.Term;

public class RationalTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof RationalTerm);
	}

}
