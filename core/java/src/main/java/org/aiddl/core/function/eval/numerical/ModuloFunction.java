package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class ModuloFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.integer( x.get(0).getIntValue() % x.get(1).getIntValue());
	}

}
