package org.aiddl.core.function.numerical;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ModuloFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.integer( x.get(0).getIntValue() % x.get(1).getIntValue());
	}

}
