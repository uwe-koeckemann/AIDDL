package org.aiddl.core.interfaces;

import org.aiddl.core.representation.Term;

/**
 * Register Observer implementation with {@link Container} to call code on changes of specific entries.
 * @author Uwe Koeckemann
 *
 */
public interface Observer {
	/**
	 * Will be called when entry changes. 
	 * @param content updated content of the entry
	 */
	public void update( Term content );
}
