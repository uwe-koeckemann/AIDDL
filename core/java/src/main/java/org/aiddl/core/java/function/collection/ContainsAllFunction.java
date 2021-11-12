package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class ContainsAllFunction implements Function {
	@Override
	public Term apply(Term x) {
		return Term.bool(x.get(0).asCollection().containsAll(x.get(1).asCollection()));
	}
}
