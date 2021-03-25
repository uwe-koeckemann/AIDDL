package org.aiddl.core.function.eval.symbolic;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class SymbolicConcatFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.get(0).asSym().concat(x.get(1).asSym());
	}
}
