package org.aiddl.core.java.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.java.representation.KeyValueTerm;
import org.aiddl.core.java.representation.Term;
/**
 * @author Uwe Koeckemann
 *
 */
public class LockableList implements LockableCollection, List<Term> {
	
	/**
	 * Enum to determine type of list.
	 * @author Uwe Koeckemann
	 *
	 */
	public enum ListType {
		/**
		 * Internal type becomes java.util.ArrayList
		 */
		ArrayList,
		/**
		 * Internal type becomes java.util.LinkedList
		 */
		LinkedList
	};
	

	
	private List<Term> internalList;
	private Map<Term,Term> internalMap;
	private boolean isUniqueMap = true;

	boolean locked = false;
	ListType listType;
	MapType mapType;

	/**
	 * Create a new lockable list with default internal types
	 */
	public LockableList( ) {
		this(ListType.ArrayList, MapType.LinkedHashMap);
	}
	
	/**
	 * Create a new lockable list of a specified type
	 * @param listType 
	 * @param mapType 
	 */
	public LockableList( ListType listType, MapType mapType ) {
		this.listType = listType;
		this.mapType = mapType;
		if ( listType.equals(ListType.ArrayList) ) {
			internalList = new ArrayList<>();
		} else {
			internalList = new LinkedList<>();
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
	public void lock() {
		this.locked = true;
	}

	@Override
	public LockableList unlock() {
		LockableList copy = new LockableList(listType, mapType);
		copy.internalList.addAll(this);
		copy.internalMap.putAll(this.internalMap);
		return copy;
	}

	@Override
	public boolean locked() {
		return locked;
	}
	
	/**
	 * Get ListType used by this list.
	 * @return the type of list
	 */
	public ListType getListType() {
		return listType;
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
		return internalList.size();
	}

	@Override
	public boolean isEmpty() {
		return internalList.isEmpty();
	}

	@Override
	public boolean contains(Object o) {
		return this.internalList.contains(o);
	}

	@Override
	public Iterator<Term> iterator() {
		return this.internalList.iterator();
	}

	@Override
	public Object[] toArray() {
		return this.internalList.toArray();
	}

	@SuppressWarnings("hiding")
	@Override
	public <Term> Term[] toArray(Term[] a) {
		return this.internalList.toArray(a);
	}

	@Override
	public boolean add(Term e) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		} 
		if ( e == null ) {
			throw new IllegalArgumentException("Trying to add null value to lockable list (non-terms are not allowed).");
		}
		if ( e instanceof KeyValueTerm ) {
			if ( this.isUniqueMap && this.internalMap.containsKey(e.getKey()) && !this.internalMap.get(e.getKey()).equals(e.getValue()) ) {
				this.isUniqueMap = false;
			}
			this.internalMap.put(e.getKey(), e.getValue());
		}
		return this.internalList.add(e);
	}
	
	@Override
	public boolean addAll(int index, Collection<? extends Term> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		for ( Term e : c ) {
			if ( e == null ) {
				throw new IllegalArgumentException("Trying to add null value to lockable list (non-terms are not allowed).");
			}
			if ( e instanceof KeyValueTerm ) { 
				this.internalMap.put(e.getKey(), e.getValue());
			}
		}
		return this.internalList.addAll(index, c);
	}
	
	@Override
	public boolean addAll(Collection<? extends Term> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		boolean change = false;
		for ( Term e : c ) {
			change |= this.add(e);
		}
		return change;
	}
	
	@Override
	public boolean remove(Object o) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		return this.remove(o);
	}

	@Override
	public boolean containsAll(Collection<?> c) {
		return this.internalList.containsAll(c);
	}

	@Override
	public boolean removeAll(Collection<?> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		return this.internalList.removeAll(c);
	}

	@Override
	public boolean retainAll(Collection<?> c) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		for ( Term e : this.internalList) {
			if ( e instanceof KeyValueTerm && ! c.contains(e) ) {
				internalMap.remove(e.getKey());
			}
		}
		return retainAll(c);
	}

	@Override
	public void clear() {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		this.internalList.clear();		
		this.internalMap.clear();
	}



	@Override
	public Term get(int index) {
		return this.internalList.get(index);
	}

	@Override
	public Term set(int index, Term element) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		return this.internalList.set(index, element);
	}

	@Override
	public void add(int index, Term element) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		this.internalList.add(index, element);
		if ( element instanceof KeyValueTerm ) {
			this.internalMap.put(element.getKey(), element.getValue());
		}
	}

	@Override
	public Term remove(int index) {
		if ( this.locked ) {
			throw new IllegalAccessError("List is locked.");
		}
		Term removed = this.remove(index);
		
		if ( removed instanceof KeyValueTerm ) {
			this.internalMap.remove(removed.getKey());
		}
		return removed;
	}

	@Override
	public int indexOf(Object o) {
		return this.internalList.indexOf(o);
	}

	@Override
	public int lastIndexOf(Object o) {
		return lastIndexOf(o);
	}

	@Override
	public ListIterator<Term> listIterator() {
		return this.internalList.listIterator();
	}

	@Override
	public ListIterator<Term> listIterator(int index) {
		return this.internalList.listIterator(index);
	}

	@Override
	public List<Term> subList(int fromIndex, int toIndex) {
		throw new IllegalAccessError("Not implemented by lockable since it may reveal access to the internal list. This would make it impossible to lock it later on.");
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
			throw new IllegalAccessError("List is locked.");
		}
		this.internalList.add(Term.keyVal(key, value));
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
			throw new IllegalAccessError("List is locked.");
		}
		if ( !this.internalMap.containsKey(key) ) {
			this.internalList.add(Term.keyVal(key, value));
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
			throw new IllegalAccessError("List is locked.");
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
		if ( !(o instanceof LockableList) ) return false;
		LockableList s = (LockableList)o;
		return s.internalList.equals(this.internalList);
	}
	
	@Override
	public int hashCode() {
		return this.internalList.hashCode()*19;
	}
}
