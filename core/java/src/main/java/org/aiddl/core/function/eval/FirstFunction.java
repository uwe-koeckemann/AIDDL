package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class FirstFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.get(0);
	}
}
