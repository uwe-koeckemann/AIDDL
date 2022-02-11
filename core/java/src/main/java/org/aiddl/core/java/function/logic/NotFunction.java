package org.aiddl.core.java.function.logic;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class NotFunction implements Function {
	@Override
	public Term apply(Term x) {
		return Term.bool(!x.getBooleanValue());
	}		
}
