package org.aiddl.core.function.symbolic;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class SymbolicSplitFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.asSym().split();
	}
}
