package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.RealTerm;
import org.aiddl.core.java.representation.Term;

public class RealTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof RealTerm);
	}

}
