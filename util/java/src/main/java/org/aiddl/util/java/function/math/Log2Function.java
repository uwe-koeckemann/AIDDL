package org.aiddl.util.java.function.math;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class Log2Function implements Function {

	private static double log10_2 = Math.log10(2.0);
	
	@Override
	public Term apply(Term args) {
		return Term.real(Math.log10(args.getDoubleValue() / log10_2));
	}

}
