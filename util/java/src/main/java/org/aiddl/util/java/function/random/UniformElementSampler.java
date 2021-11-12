package org.aiddl.util.java.function.random;

import java.util.Random;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

/**
 * Get random element from a collection term.
 * 
 * @author Uwe Koeckemann
 *
 */
public class UniformElementSampler implements Function {

	Random r = new Random();
	
	@Override
	public Term apply(Term args) {
		return args.asList().get(r.nextInt(args.size()));
	}

}
