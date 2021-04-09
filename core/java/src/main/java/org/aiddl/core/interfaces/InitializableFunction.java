package org.aiddl.core.interfaces;

import org.aiddl.core.representation.Term;

/**
 * Interface for function that can be initialized
 * @author Uwe Koeckemann
 *
 */
public interface InitializableFunction extends Function {
	
	/**
	 * Initialize a function with initial data.
	 * @param args arguments for initialization
	 */
	public void initialize( Term args );
	
}
