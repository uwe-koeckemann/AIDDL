package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.BooleanTerm;
import org.aiddl.core.representation.Term;

public class BooleanTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof BooleanTerm);
	}

}
