package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.RationalTerm;
import org.aiddl.core.java.representation.Term;

public class RationalTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof RationalTerm);
	}

}
