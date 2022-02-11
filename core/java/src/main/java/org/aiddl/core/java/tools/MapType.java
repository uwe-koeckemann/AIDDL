package org.aiddl.core.java.tools;
/**
 * Enum to determine type of internal map for lockable data structures.
 * @author Uwe Koeckemann
 *
 */
public enum MapType {
	/**
	 * Internal type becomes java.util.ArrayList
	 */
	HashMap,
	/**
	 * Internal type becomes java.util.LinkedList
	 */
	LinkedHashMap
}

