package org.aiddl.core.function.eval.symbolic;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class SymbolicSplitFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.asSym().split();
	}
}
