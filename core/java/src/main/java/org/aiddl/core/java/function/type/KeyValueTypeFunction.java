package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.KeyValueTerm;
import org.aiddl.core.java.representation.Term;

public class KeyValueTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof KeyValueTerm);
	}

}
