package org.aiddl.util.java.function.math;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class LnFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.log10(args.getDoubleValue()));
	}

}
