package org.aiddl.core.java.function.symbolic;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class SymbolicConcatFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(0).asSym().concat(x.get(1).asSym());
	}
}
