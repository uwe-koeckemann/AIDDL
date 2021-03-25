package org.aiddl.core.tools;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.Term;

/**
 * @author Uwe Köckemann
 *
 */
public class LockableSet implements LockableCollection, Set<Term> {

	/**
	 * Enum to determine type of list.
	 * @author Uwe Köckemann
	 *
	 */
	public enum SetType {
		/**
		 * Internal type becomes java.util.ArrayList
		 */
		HashSet,
		/**
		 * Internal type becomes java.util.LinkedList
		 */
		LinkedHashSet
	};
		
	private Set<Term> internalSet;
	private Map<Term,Term> internalMap;
	private boolean isUniqueMap = true;
		
	boolean locked = false;
	SetType setType;
	MapType mapType;
	
	/**
	 * Create new lockable set with default internal types.
	 */
	public LockableSet() {
		this(SetType.LinkedHashSet, MapType.LinkedHashMap);
	}
	
	/**
	 * Create a new lockable list of a specified type
	 * @param setType 
	 * @param mapType 
	 */
	public LockableSet( SetType setType, MapType mapType ) {
		this.setType = setType;
		this.mapType = mapType;
		if ( setType.equals(SetType.HashSet) ) {
			internalSet= new HashSet<>();
		} else {
			internalSet= new LinkedHashSet<>();
		}
		if ( mapType.equals(MapType.HashMap) ) {
			internalMap = new HashMap<>();
		} else {
			internalMap = new LinkedHashMap<>();
		}
	}
	
	/** 
	 * Check if the map contained in this collection is unique 
	 * @return <code>true</code> if this collection assigns a unique value to each key, <code>false</code> otherwise
	 */
	public boolean isUniqueMap() {
		return this.isUniqueMap;
	}
	
	@Override
	public boolean locked() {
 		return this.locked;
	}

	@Override
	public void lock() {
		this.locked = true;
	}

	@Override
	public LockableSet unlock() {
		LockableSet copy = new LockableSet(this.setType, this.mapType);
		copy.internalSet.addAll(this.internalSet);
		copy.internalMap.putAll(this.internalMap);
		return copy;
	}
	/**
	 * Get ListType used by this list.
	 * @return the type of list
	 */
	public SetType getSetType() {
		return setType;
	}
	
	/**
	 * get MapType used by this list
	 * @return the map type
	 */
	public MapType getMapType() {
		return mapType;
	}
	@Override
	public int size() {
		return internalSet.size();
	}

	@Override
	public boolean isEmpty() {
		return internalSet.isEmpty();
	}

	@Override
	public boolean contains(Object o) {
		return internalSet.contains(o);
	}

	@Override
	public Iterator<Term> iterator() {
		return internalSet.iterator();
	}

	@Override
	public Object[] toArray() {
		return internalSet.toArray();
	}

	@Override
	public <T> T[] toArray(T[] a) {
		return internalSet.toArray(a);
	}

	@Override
	public boolean add(Term e) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		if ( e == null ) {
			throw new IllegalArgumentException("Trying to add null value to lockable set (non-terms are not allowed).");
		}
		if ( e instanceof KeyValueTerm ) {
			if ( this.isUniqueMap && this.internalMap.containsKey(e.getKey()) && !this.internalMap.get(e.getKey()).equals(e.getValue()) ) {
				this.isUniqueMap = false;
			}
			this.internalMap.put(e.getKey(), e.getValue());
		}
		return this.internalSet.add(e);
	}

	@Override
	public boolean remove(Object o) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		if ( o instanceof KeyValueTerm ) {
			this.internalMap.remove(((KeyValueTerm)o).getKey());
		}
		return this.internalSet.remove(o);
	}

	@Override
	public boolean containsAll(Collection<?> c) {
		return this.internalSet.containsAll(c);
	}

	@Override
	public boolean addAll(Collection<? extends Term> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		boolean change = false;
		for ( Term e : c ) {
			change |= this.add(e);
		}
		return change;
	}

	@Override
	public boolean removeAll(Collection<?> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		boolean change = false;
		for ( Object e : c ) {
			change |= this.remove(e);
		}
		return change;
	}

	@Override
	public boolean retainAll(Collection<?> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		for ( Term e : this.internalSet ) {
			if ( e instanceof KeyValueTerm && ! c.contains(e) ) {
				internalMap.remove(e.getKey());
			}
		}
		return this.retainAll(c);
	}

	@Override
	public void clear() {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		this.internalSet.clear();
		this.internalMap.clear();
	}
	
	/**
	 * See {@link Map}
	 * @param key
	 * @return See {@link Map}
	 */
	public boolean containsKey(Term key) {
		return this.internalMap.containsKey(key);
	}

	/**
	 * See {@link Map}
	 * @param value
	 * @return See {@link Map}
	 */
	public boolean containsValue(Term value) {
		return this.internalMap.containsValue(value);
	}

	/**
	 * See {@link Map}
	 * @param key
	 * @return See {@link Map}
	 */
	public Term get(Term key) {
		return  this.internalMap.get(key);
	}
	/**
	 * Returns value of key of a key value term with key. If that key does not exist return a default value.
	 * @param key
	 * @param defaultValue 
	 * @return value belonging to key or default value if the key does not exist
	 */
	public Term getOrDefault(Term key, Term defaultValue) {
		Term r = this.internalMap.get(key);
		if ( r != null ) {
			return r;
		}
		return defaultValue;
	}

	/**
	 * Put an entry into the internal map. Also adds a key-value pair term to the internal list. See {@link Map}
	 * @param key
	 * @param value
	 * @return See {@link Map}
	 */
	public Term put(Term key, Term value) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		this.internalSet.add(Term.keyVal(key, value));
		return this.internalMap.put(key, value);
	}
	
	/**
	 * Add a key value pair if no such pair with the given key already exists.
	 * @param key the key term 
	 * @param value the value term
	 * @return <code>null</code> if the key did not exist, <code>value</code> otherwise
	 */
	public Term putIfAbsent(Term key, Term value) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		if ( !this.internalMap.containsKey(key) ) {
			this.internalSet.add(Term.keyVal(key, value));
			return this.internalMap.put(key, value);
		} else {
			return null;
		}
	}

	/**
	 * Use put() for all entries in map m. See {@link Map}.
	 * @param m see {@link Map}
	 */
	public void putAll(Map<? extends Term, ? extends Term> m) {
		if ( this.locked ) {
			throw new IllegalAccessError("Set is locked.");
		}
		for ( Term key : m.keySet() ) {
			this.put(key, m.get(key));
		}
	}

	/**
	 * See {@link Map}
	 * @return see {@link Map}
	 */
	public Set<Term> keySet() {
		return this.internalMap.keySet();
	}

	/**
	 * See {@link Map}
	 * @return see {@link Map}
	 */
	public Collection<Term> values() {
		return this.internalMap.values();
	}
	
	@Override
	public boolean equals( Object o ) {
		if ( o == this ) return true;
		if ( !(o instanceof LockableSet) ) return false;
		LockableSet s = (LockableSet)o;
		return s.internalSet.equals(this.internalSet);
	}
	
	@Override
	public int hashCode() {
		return this.internalSet.hashCode()*17;
	}
}
