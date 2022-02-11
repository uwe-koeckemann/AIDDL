package org.aiddl.util.java.function.random;

import java.util.Random;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class UniformRealSampler implements Function {

	Random r = new Random();
	
	@Override
	public Term apply(Term args) {
		return Term.real(r.nextDouble());
	}

}
