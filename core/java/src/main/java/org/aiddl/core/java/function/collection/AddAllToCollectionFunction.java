package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class AddAllToCollectionFunction implements Function {
	@Override
	public Term apply(Term x) {
		return x.get(0).asCollection().addAll(x.get(1).asCollection());
	}
}
