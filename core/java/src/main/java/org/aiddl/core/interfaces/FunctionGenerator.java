package org.aiddl.core.interfaces;

/**
 * An object that can provide a {@link Function}. 
 * 
 * @author Uwe Köckemann
 *
 */
public interface FunctionGenerator {

	/**
	 * Generate a function. 
	 * @return a function
	 */
	public Function generate();
	
}
