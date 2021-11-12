package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.VariableTerm;

public class VariableTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof VariableTerm);
	}

}
