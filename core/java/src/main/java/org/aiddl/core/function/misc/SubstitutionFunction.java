package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

public class SubstitutionFunction implements Function {

	@Override
	public Term apply(Term x) {
		Substitution s = new Substitution(x.get(1));
		return x.get(0).substitute(s);
	}
}
