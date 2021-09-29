package org.aiddl.util.function.random;

import java.util.Random;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class UniformIntegerSampler implements Function {

	Random r = new Random();
	
	@Override
	public Term apply(Term args) {
		int min = args.get(0).getIntValue();
		int max = args.get(1).getIntValue();
		return Term.integer(min + r.nextInt(max-min));
	}

}
