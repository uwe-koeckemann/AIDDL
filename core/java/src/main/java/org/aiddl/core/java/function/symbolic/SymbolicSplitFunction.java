package org.aiddl.core.java.function.symbolic;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class SymbolicSplitFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.asSym().split();
	}
}
