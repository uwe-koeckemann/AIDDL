package org.aiddl.core.function.eval.math;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ArcCosineFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.acos(args.getDoubleValue()));
	}

}
