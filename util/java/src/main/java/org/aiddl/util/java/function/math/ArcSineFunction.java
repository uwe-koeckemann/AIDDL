package org.aiddl.util.java.function.math;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class ArcSineFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.asin(args.getDoubleValue()));
	}

}
