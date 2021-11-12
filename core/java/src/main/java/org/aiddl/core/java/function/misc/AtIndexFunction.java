package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class AtIndexFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(1).get(x.get(0).getIntValue());
	}
}
