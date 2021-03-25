package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;

public class SetTypeFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof SetTerm);
	}

}
