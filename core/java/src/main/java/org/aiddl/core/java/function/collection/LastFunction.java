package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class LastFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(x.size()-1);
	}
}
