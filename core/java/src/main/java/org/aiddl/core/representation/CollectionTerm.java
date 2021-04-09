package org.aiddl.core.representation;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * Term that is a collection of terms (i.e., list or set).
 * This allows to iterate over lists and sets without casting
 * to ListTerm or SetTerm.
 *
 * @author Uwe Koeckemann
 */
public abstract class CollectionTerm extends Term implements Iterable<Term> {

	/** 
	 * Check if collection term contains a term
	 * @param t a term
	 * @return <code>true</code> if collection term contains term, <code>false</code> otherwise
	 */
	public abstract boolean contains( Term t );
	
	/** 
	 * Check if collection term contains all terms of another collection term.
	 * @param S a collection term term
	 * @return <code>true</code> if collection term contains all term in <code>S</code>, <code>false</code> otherwise
	 */
	public abstract boolean containsAll( CollectionTerm S );
	
	/** 
	 * Check if collection term contains any term of another collection term.
	 * @param S a collection term term
	 * @return <code>true</code> if collection term contains any term in <code>S</code>, <code>false</code> otherwise
	 */
	public abstract boolean containsAny( CollectionTerm S );
	
	/** 
	 * Check if this term has a key value pair with this key. 
	 * @param key 
	 * @return <code>true</code> if this collection has a key value term with given key, <code>false</code> otherwise
	 */
	public abstract boolean containsKey( Term key );
	
	/** 
	 * Check if the map contained in this collection is unique 
	 * @return <code>true</code> if this collection assigns a unique value to each key, <code>false</code> otherwise
	 */
	public abstract boolean isUniqueMap();
	
	/** 
	 * Add a term to a collection term.
	 * @param t a term to add
	 * @return collection with <code>t</code> 
	 */
	public abstract CollectionTerm add( Term t );
	
	/** 
	 * Merge key-value pairs of this map and another into a new collection term retaining only key-value pairs.
	 * @param t another collection term term
	 * @return a new collection term containing the map of this after overwriting it with the map of t 
	 */
	public abstract CollectionTerm putAll( CollectionTerm t );
	
	/** 
	 * Add a collection of terms to this collection term.
	 * @param t a term to add
	 * @return collection with <code>t</code> 
	 */
	public abstract CollectionTerm addAll( CollectionTerm t );
	
	/** 
	 * Remove a term from a collection term.
	 * @param t a term to remove
	 * @return collection without <code>t</code> 
	 */
	public abstract CollectionTerm remove( Term t );
	
	/** 
	 * Remove all terms in a collection from this collection.
	 * @param t a collection of terms to remove
	 * @return collection without terms in <code>t</code> 
	 */
	public abstract CollectionTerm removeAll( CollectionTerm t );
	
	/** 
	 * Check if this collection is empty.
	 * @return <code>true</code> if collection does not contain any elements, <code>false</code> otherwise 
	 */
	public abstract boolean isEmpty();
	

	/** Add all terms in this collection term to the argument.
	 * This method allows to add to a collection without copying this 
	 * term into a collection term first.
	 * @param S collection we want to add to
	 */
	public void addAllTo( Collection<Term> S ) {
		this.forEach((element)->{S.add(element);});
	}
		
	/**
	 * Take all {@link KeyValueTerm}s in this collection and add them to a
	 * {@link Map}.
	 * @return the map
	 */
	public Map<Term,Term> getMap( ) {
		Map<Term,Term> r = new HashMap<>();
		for ( Term element : this ) {
			if ( element instanceof KeyValueTerm ) {
				r.put(element.getKey(), element.getValue());
			}
		}		
		return r;
	}
}
