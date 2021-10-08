package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class RemoveAllFromCollectionFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(0).asCollection().removeAll(x.get(1).asCollection());
	}
}
