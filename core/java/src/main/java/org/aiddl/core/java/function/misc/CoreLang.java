package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

/**
 * Returns Symbol representing language of this core implementation
 * 
 * @author Uwe Koeckemann
  */
public class CoreLang implements Function {

	private static SymbolicTerm Lang = Term.sym("java");
	
	@Override
	public Term apply(Term args) {
		return Lang;
	}

}
