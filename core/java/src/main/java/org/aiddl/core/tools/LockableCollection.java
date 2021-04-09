package org.aiddl.core.tools;

import java.util.Collection;

import org.aiddl.core.representation.Term;

/**
 * Adding lock methods to the collection interface.
 * Once the lock() method is called, the collection becomes immutable and any attempt to change its contents will throw an exception.
 * The purpose of lockable collections is to allow creating and manipulating an object that will become part of an immutable data structure
 * afterwards. 
 * 
 * @author Uwe Koeckemann
 */
public interface LockableCollection extends Collection<Term> {
	
	/**
	 * Check if this collection is locker.
	 * @return <code>true</code> if the collection is locked, <code>false</code> otherwise
	 */
	boolean locked();
	
	/**
	 * Lock this collection. After this method is called, the collection cannot be changed anymore.
	 */
	void lock();
	
	/**
	 * Unlock the collection. This method returns an unlocked copy of the (possibly) locked collection.  
	 * @return copy of collection that is not locked
	 */
	Collection<Term> unlock();
}
