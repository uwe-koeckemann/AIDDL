package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class AtIndexFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(1).get(x.get(0).getIntValue());
	}
}
