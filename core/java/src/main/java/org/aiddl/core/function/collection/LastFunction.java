package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class LastFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(x.size()-1);
	}
}
