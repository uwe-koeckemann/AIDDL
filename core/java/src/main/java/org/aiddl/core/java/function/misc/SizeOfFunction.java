package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class SizeOfFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.integer(x.size());
	}
}
