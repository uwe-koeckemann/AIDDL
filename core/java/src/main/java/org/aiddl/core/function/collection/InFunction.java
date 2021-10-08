package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class InFunction implements Function {
	@Override
	public Term apply(Term x) {
		return Term.bool(x.get(1).asCollection().contains(x.get(0)));
	}		
}
