package org.aiddl.core.java.function.numerical;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class ModuloFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.integer( x.get(0).getIntValue() % x.get(1).getIntValue());
	}

}
