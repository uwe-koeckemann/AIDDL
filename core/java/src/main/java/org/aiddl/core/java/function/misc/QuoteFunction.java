package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.Term;

public class QuoteFunction implements LazyFunction {

	@Override
	public Term apply(Term x) {
		return x;
	}

}
