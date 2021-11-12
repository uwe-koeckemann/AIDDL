package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class PutAllFunction implements Function {

	@Override
	public Term apply(Term t) {
		return t.get(0).asCollection().putAll(t.get(1).asCollection() );
	}

}
