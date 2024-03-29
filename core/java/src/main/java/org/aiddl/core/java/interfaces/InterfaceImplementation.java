package org.aiddl.core.java.interfaces;

import org.aiddl.core.java.representation.SymbolicTerm;

/**
 * An extension of Function that contains a declaration.
 * @author Uwe Koeckemann
 */
public interface InterfaceImplementation extends Function {
	/**
	 * Get function declaration containing its URI and allowing to check types of input and output arguments.
	 * @return function declaration
	 */
	SymbolicTerm getInterfaceUri();
}
