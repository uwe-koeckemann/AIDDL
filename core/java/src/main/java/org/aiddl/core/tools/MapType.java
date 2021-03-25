package org.aiddl.core.tools;
/**
 * Enum to determine type of internal map for lockable data structures.
 * @author Uwe Köckemann
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

