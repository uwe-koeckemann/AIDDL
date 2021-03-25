package org.aiddl.core.function.eval.random;

import java.util.Random;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class UniformRealSampler implements Function {

	Random r = new Random();
	
	@Override
	public Term apply(Term args) {
		return Term.real(r.nextDouble());
	}

}
