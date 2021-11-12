package org.aiddl.util.java.function.random;

import java.util.Random;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class NormalDistributionSampler implements Function {

	Random r = new Random();
	
	@Override
	public Term apply(Term args) {
		double mean = args.get(0).asReal().getDoubleValue();
		double std = args.get(1).asReal().getDoubleValue();
		double value = mean + r.nextGaussian()*std;
		return Term.real(value);
	}

}
