package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

public class SubstitutionFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		Substitution s = new Substitution(x.get(1));
		return x.get(0).substitute(s);
	}
}
