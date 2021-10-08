package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.VariableTerm;

public class VariableTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof VariableTerm);
	}

}
