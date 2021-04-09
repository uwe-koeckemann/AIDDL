package org.aiddl.core.tools;

/**
 * Static class that provides unique IDs.
 * Mostly used to make interval terms and variable terms unique.
 *  
 * @author Uwe Koeckemann
 */
public class UniqueID {
	static private long ID = 0;
	
	/**
	 * Increment ID and return new one.
	 * @return a new ID
	 */
	public static long getID() {
		return ++ID;
	}
	/**
	 * Get the last used ID (without incrementing).
	 * @return the previously used ID
	 */
	public static long getLastID() {
		return ID;
	}
	
	/**
	 * Reset internal ID to 0 (stores old value).
	 */
	public static void reset() {
		ID = 0;
	}
}

