package org.aiddl.util.function.math;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ArcTangentFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.atan(args.getDoubleValue()));
	}

}
