package org.aiddl.core.java.interfaces;

import java.util.Map;

import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.representation.Term;

/**
 * Interface for function that can be configured
 * @author Uwe Koeckemann
 *
 */
public interface ConfigurableFunction extends Function {
	
	/**
	 * Configure a function.
	 * @param settings map from options to values
	 * @param fReg a function registry used to load functions needed by other functions
	 */
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg );
}
