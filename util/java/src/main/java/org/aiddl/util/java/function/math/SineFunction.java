package org.aiddl.util.java.function.math;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class SineFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.sin(args.getDoubleValue()));
	}

}
