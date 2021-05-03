package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class PutAllFunction implements Function {

	@Override
	public Term apply(Term t) {
		return t.get(0).asCollection().putAll(t.get(1).asCollection() );
	}

}
