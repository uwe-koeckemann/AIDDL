package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class SizeOfFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.integer(x.size());
	}
}
