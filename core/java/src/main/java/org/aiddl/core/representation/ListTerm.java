package org.aiddl.core.representation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.aiddl.core.container.Container;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableList.ListType;
import org.aiddl.core.tools.MapType;

/** Term representing a list of terms.
 * @author Uwe Koeckemann
 *
 */
public class ListTerm extends CollectionTerm implements Iterable<Term> {
	LockableList values;
		
	Integer hashCode = null;
	
	int first;
	int last;

	
	protected final static ListTerm Empty = new ListTerm();
	
	protected ListTerm( Term... values ) {
		this.values = new LockableList(ListType.ArrayList, MapType.HashMap);
		for ( int i = 0 ; i < values.length ; i++ ) {
			this.values.add(values[i]);
		}
		this.values.lock();
		this.first = 0;
		this.last = this.values.size();
	}	
	
	protected ListTerm( Collection<? extends Term> values ) {
		this.values = new LockableList(ListType.ArrayList, MapType.HashMap);
		for ( Term t : values ) {
			this.values.add(t);
		}
		this.values.lock();
		this.first = 0;
		this.last = this.values.size();		
	}	
	
	protected ListTerm( LockableList values ) {
		this.values = values;
		this.values.lock();
		this.first = 0;
		this.last = this.values.size();		
	}	
	
	protected ListTerm( Map<Term,Term> values ) {
		this.values = new LockableList(ListType.ArrayList, MapType.HashMap);
		for ( Term k : values.keySet() ) {
			Term v = values.get(k);
			this.values.add(Term.keyVal(k, v));
			this.values.put(k,v);
		}
		this.values.lock();
		this.first = 0;
		this.last = this.values.size();
	}	
	
	private ListTerm( LockableList values, int first, int last ) {
		this.values = values;
		this.values.lock();
		
//		if ( first < 0 || last > this.values.size() ) {
//			throw new IllegalArgumentException("Can only shrink list.");
//		}
		
		this.first = first;
		this.last = last;		
	}	
	
	/**
	 * Get the locked version of the internal list.
	 * Any attempt to change this locked list will
	 * cause an exception. This allows to access
	 * the internal collection without the need
	 * to copy it.
	 * 
	 * @return LockableList in locked state
	 */
	public LockableList getLockedList() {
		return this.values;
	}
	
	@Override
	public Integer size() {
		return this.last - this.first; 
	}
	
	@Override
	public Term get( int i ) {
		return this.values.get(this.first+i);
	}
	
	@Override
	public Term get( Term t ) {
		return this.values.get(t);
	}
	
	@Override
	public Term getOrDefault( Term key, Term defaultValue ) {
		return this.values.getOrDefault(key, defaultValue);
	}	
	
	@Override 
	public boolean containsKey( Term key ) {
		return this.values.containsKey(key); 
	}
	
	@Override
	public boolean isUniqueMap() {
		return this.values.isUniqueMap();
	}
	
	/** 
	 * Get index of first appearance of element t in this list term.
	 * @param t the term we are looking for
	 * @return index of first appearance of t or -1 if t does not appear in this list
	 */
	public int indexOf ( Term t ) {
		for ( int i = this.first ; i < this.last ; i++ ) {
			if ( t.equals(this.values.get(i))) {
				return i;
			}
		}
		return -1;
	}
	
	/**
	 * Get first element of this list.
	 * @return the first element
	 */
	public Term getFirst() {
		if ( this.size() > 0 ) {
			return this.get(0);
		}
		throw new IllegalAccessError("Empty list has no first element.");
	}
	
	/**
	 * Get last element of this list.
	 * @return the last element
	 */
	public Term getLast() {
		int s = this.size();
		if ( s > 0 ) {
			return this.get(s-1);
		}
		throw new IllegalAccessError("Empty list has no last element.");
	}
	
	/**
	 * Remove first element from list. This operation takes O(1)
	 * time unless the removed element is a key value term (in this
	 * case the internal list has to be reconstructed). 
	 * @return list term without the first element
	 */
	public ListTerm removeFirst() {
		if ( this.size() > 0 ) {
			if ( this.values.get(this.first) instanceof KeyValueTerm ) {
				LockableList new_list = new LockableList();
				for ( int i = first+1 ; i < this.last ; i++ ) {
					new_list.add(this.values.get(i));
				}
				return new ListTerm(new_list);
			}
			return new ListTerm(this.values, this.first+1, this.last);
		}
		throw new IllegalAccessError("Empty list has no first element.");
	}
	
	/**
	 * Remove last element from list. This operation takes O(1)
	 * time unless the removed element is a key value term (in this
	 * case the internal list has to be reconstructed). 
	 * @return list term without the first element
	 */
	public ListTerm removeLast() {
		if ( this.size() > 0 ) {
			if ( this.values.get(this.last-1) instanceof KeyValueTerm ) {
				LockableList new_list = new LockableList();
				for ( int i = first ; i < this.last-1 ; i++ ) {
					new_list.add(this.values.get(i));
				}
				return new ListTerm(new_list);
			}
			return new ListTerm(this.values, this.first, this.last-1);
		} 
		throw new IllegalAccessError("Empty list has no last element.");
	}
	
	@Override
	public ListTerm add( Term t ) {
		LockableList new_values = this.values.unlock();
		new_values.add(t);
		return Term.list(new_values);
	}
	
	@Override
	public ListTerm addAll( CollectionTerm C ) {
		LockableList new_values = this.values.unlock();
		for ( Term e : C ) {
			new_values.add(e);
		}
		return Term.list(new_values);
	}
	
	@Override
	public ListTerm putAll( CollectionTerm t ) {
		LockableList copy = new LockableList(this.values.getListType(), this.values.getMapType()); 
		for ( Term key : this.values.keySet() ) {
			if ( !t.containsKey(key) ) {
				copy.put(key, this.values.get(key));
			}
		}
		for ( Term kvp : t ) {
			if ( kvp instanceof KeyValueTerm ) {
				copy.put(kvp.getKey(), kvp.getValue());
			}			
		}
		return Term.list(copy);
	}
	
	@Override
	public CollectionTerm remove(Term t) {
		LockableList copy = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			if ( !v.equals(t) ) {
				copy.add(v);
			}
		}
		return Term.list(copy);
	}
	
	@Override
	public CollectionTerm removeAll(CollectionTerm c) {
		LockableList copy = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			if ( !c.contains(v) ) {
				copy.add(v);
			}
		}
		return Term.list(copy);
	}
	
	@Override
	public ListTerm asList() {
		return this;
	}
	
	@Override
	public SetTerm asSet() {
		return Term.set(values);
	}
	
	@Override
	public ListTerm asCollection() {
		return this;
	}
	
	@Override
	public List<Term> getListCopy() {
		List<Term> L = new ArrayList<Term>();
		for ( Term t : values ) {
			L.add(t);
		}
		return L;
	}
	
	@Override
	public List<Term> getCollectionCopy() {
		return this.getListCopy();
	}
		
	@Override
	public boolean contains ( Term t ) {
		for ( Term e : this ) {
			if ( e.equals(t) ) {
				return true;
			}
		}
		return false;
	}
	
	@Override
	public boolean containsAll( CollectionTerm S ) {
		for ( Term s : S ) {
			if ( !this.contains(s) ) {
				return false;
			}
		}
		return true;
	}
	
	@Override
	public boolean containsAny( CollectionTerm S ) {
		for ( Term s : S ) {
			if ( this.contains(s) ) {
				return true;
			}
		}
		return false;
	}

	@Override
	public boolean isEmpty() {
		return this.values.isEmpty();
	}
		
	@Override
	public Substitution match(Term t) {
		if ( t instanceof ListTerm ) {
			Substitution sCombined = new Substitution();
			ListTerm tList = (ListTerm)t;
			
			if ( this.values.size() != tList.values.size() ) {
				return null;
			}
			
			for ( int i = 0 ; i < this.values.size() ; i++ ) {
				Substitution subArg = this.values.get(i).match(tList.values.get(i));
				if ( subArg == null ) {
					return null;
				}
				if ( !sCombined.add(subArg) ) {
					return null;
				}
			}
			return sCombined;
			
		}
		
		return super.match(t);
	}
	
	@Override
	public ListTerm substitute(Substitution s) {
		LockableList s_new = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.substitute(s));
		}		
		return Term.list(s_new);
	}
	
	@Override
	public ListTerm resolve(Container db) {
		LockableList s_new = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.resolve(db));
		}		
		return Term.list(s_new);
	}
		
	@Override
	public boolean isGround() {
		for ( Term t : this.values ) {
			if ( !t.isGround() ) {
				return false;
			}
		}
		return true;
	}
	
	@Override
	public boolean isComplex() {
		return true;
	}

	@Override
	public boolean isVariable() {
		return false;
	}

	@Override
	public boolean isConstant() {
		return false;
	}

	@Override
	public String toString() {
		StringBuilder sB = new StringBuilder();
		sB.append("[");
		if ( this.values.size() > 0 ) 
			sB.append(values.get(0));
		for ( int i = 1 ; i < values.size(); i++ ) {
			sB.append(" ");
			sB.append(values.get(i).toString());
		}
		sB.append("]");
		return sB.toString();
	}
	

	@Override
	public int hashCode() {
		if ( this.hashCode == null ) {
			this.hashCode = 7*this.values.hashCode();
		}
		return this.hashCode;
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof ListTerm) ) {
    		return false;
    	}
    	
    	ListTerm t = (ListTerm)o;
    	if ( t.values.size() != this.values.size() ) {
    		return false;
    	}
		for ( int i = 0 ; i < t.values.size() ; i++ ) {
			if ( !this.values.get(i).equals(t.values.get(i)) ) {
				return false;
			}
		}
    	return true; 
    }

	@Override
	public Iterator<Term> iterator() {
		return this.values.iterator();
	}
}
