package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class ValueFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.getValue();
	}
}
