package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ContainsFunction implements Function {
	@Override
	public Term apply(Term x) {
		return Term.bool(x.get(0).asCollection().contains(x.get(1)));
	}
}
