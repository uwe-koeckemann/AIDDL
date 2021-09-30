package org.aiddl.util.function.math;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class TangentFunction implements Function {

	@Override
	public Term apply(Term args) {
		return Term.real(Math.tan(args.getDoubleValue()));
	}

}
