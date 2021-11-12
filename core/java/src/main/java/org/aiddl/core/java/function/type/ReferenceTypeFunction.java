package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.ReferenceTerm;
import org.aiddl.core.java.representation.Term;

public class ReferenceTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof ReferenceTerm);
	}

}
