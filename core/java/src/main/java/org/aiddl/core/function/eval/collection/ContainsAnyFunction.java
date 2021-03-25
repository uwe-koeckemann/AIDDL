package org.aiddl.core.function.eval.collection;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class ContainsAnyFunction implements PureFunction {
	@Override
	public Term apply(Term x) {
		return Term.bool(x.get(0).asCollection().containsAny(x.get(1).asCollection()));
	}
}
