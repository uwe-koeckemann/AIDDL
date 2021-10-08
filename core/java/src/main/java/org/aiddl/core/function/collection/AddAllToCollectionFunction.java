package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class AddAllToCollectionFunction implements Function {
	@Override
	public Term apply(Term x) {
		return x.get(0).asCollection().addAll(x.get(1).asCollection());
	}
}
