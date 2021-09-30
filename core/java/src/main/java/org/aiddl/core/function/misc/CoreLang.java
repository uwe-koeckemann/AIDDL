package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

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
