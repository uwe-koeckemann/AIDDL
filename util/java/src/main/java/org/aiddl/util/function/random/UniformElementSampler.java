package org.aiddl.util.function.random;

import java.util.Random;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

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
