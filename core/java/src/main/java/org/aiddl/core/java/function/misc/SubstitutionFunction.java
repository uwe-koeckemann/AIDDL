package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;

public class SubstitutionFunction implements Function {

	@Override
	public Term apply(Term x) {
		Substitution s = new Substitution(x.get(1));
		return x.get(0).substitute(s);
	}
}
