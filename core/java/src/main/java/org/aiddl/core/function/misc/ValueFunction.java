package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ValueFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.getValue();
	}
}
