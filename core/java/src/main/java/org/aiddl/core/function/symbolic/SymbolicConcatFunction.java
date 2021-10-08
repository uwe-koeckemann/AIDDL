package org.aiddl.core.function.symbolic;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class SymbolicConcatFunction implements Function {

	@Override
	public Term apply(Term x) {
		return x.get(0).asSym().concat(x.get(1).asSym());
	}
}
