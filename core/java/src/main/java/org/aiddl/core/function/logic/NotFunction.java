package org.aiddl.core.function.logic;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class NotFunction implements Function {
	@Override
	public Term apply(Term x) {
		return Term.bool(!x.getBooleanValue());
	}		
}
