package org.aiddl.core.java.representation;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.tools.LockableSet;
import org.aiddl.core.java.tools.MapType;

/** A term representing a set of terms.
 * @author Uwe Koeckemann
 *
 */
public class SetTerm extends CollectionTerm implements Iterable<Term> {
	LockableSet values;
	Integer hashCode = null;
	
	protected final static SetTerm Empty = new SetTerm();
			
	protected SetTerm( Term... values ) {
		this.values = new LockableSet(LockableSet.SetType.LinkedHashSet, MapType.LinkedHashMap);
		for ( int i = 0 ; i < values.length ; i++ ) {
			this.values.add(values[i]);
			if ( values[i] instanceof KeyValueTerm ) {
				this.values.put(values[i].getKey(), values[i].getValue());
			}
		}
		this.values.lock();
	}
	
	protected SetTerm( Collection<? extends Term> values ) {
		this.values = new LockableSet(LockableSet.SetType.LinkedHashSet, MapType.LinkedHashMap);
		for ( Term v : values ) {
			this.values.add(v);
		}
		this.values.lock();
	}
	
	protected SetTerm( LockableSet values ) {
		this.values = values;
		this.values.lock();
	}
	
	protected SetTerm( Map<Term,Term> values ) {
		this.values = new LockableSet(LockableSet.SetType.LinkedHashSet, MapType.LinkedHashMap);
		this.values.putAll(values);
		this.values.lock();
	}
		
	/**
	 * Get the locked version of the internal set.
	 * Any attempt to change this locked set will
	 * cause an exception. This allows to access
	 * the internal collection without the need
	 * to copy it.
	 * 
	 * @return LockableSet in locked state
	 */
	public LockableSet getLockedSet() {
		return this.values;
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
	
	@Override
	public SetTerm add( Term t ) {
		LockableSet copy = this.values.unlock();
		copy.add(t);
		return Term.set(copy);
	}
	
	@Override
	public SetTerm addAll( CollectionTerm t ) {
		LockableSet copy = this.values.unlock();
		for ( Term v : t ) {
			copy.add(v);
			
		}	
		return Term.set(copy);
	}
	
	@Override
	public SetTerm putAll( CollectionTerm t ) {
		LockableSet copy = new LockableSet(this.values.getSetType(), this.values.getMapType()); 
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
		return Term.set(copy);
	}
	
	@Override
	public CollectionTerm remove(Term t) {
		LockableSet copy = this.values.unlock();
		copy.remove(t);
		return Term.set(copy);
	}
	@Override
	public CollectionTerm removeAll(CollectionTerm c) {
		LockableSet copy = this.values.unlock();
		for ( Term e : c )
			copy.remove(e);
		return Term.set(copy);
	}
	
	@Override
	public boolean isEmpty() {
		return this.values.isEmpty();
	}
	
	@Override
	public boolean contains( Term t ) {		
		return this.values.contains(t);
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
	public Integer size() {
		return this.values.size();
	}

	@Override
	public SetTerm asSet() {
		return this;
	}
	
	@Override
	public Set<Term> getSetCopy() {
		Set<Term> r = new LinkedHashSet<Term>();
		for ( Term t : this.values ) {
			r.add(t);
		}
		return r;
	}
	@Override
	public ListTerm asList() {
		return Term.list(this.values);
	}
	@Override
	public SetTerm asCollection() {
		return this;
	}
	@Override
	public Collection<Term> getCollectionCopy() {
		return this.getSetCopy();
	}
	
	@Override
	public Substitution match(Term t) {
		// Sets never match unless they are equal 
		// Reason: matching sets is difficult and does not necessarily have a unique solution
		if ( this.equals(t) ) {
			return new Substitution();
		}
		return super.match(t);
	}
	
	@Override
	public SetTerm substitute(Substitution s) {
		LockableSet s_new = new LockableSet(this.values.getSetType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.substitute(s));
		}		
		return Term.set(s_new);
	}
	
	@Override
	public SetTerm resolve(Container db) {
		LockableSet s_new = new LockableSet(this.values.getSetType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.resolve(db));
		}		
		return Term.set(s_new);
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
		sB.append("{");
		int i = 0;
		for ( Term t : this.values ) {
			sB.append(t);
			if ( i < values.size()-1 ) 
				sB.append(" ");
			i++;
		}
		sB.append("}");
		
		return sB.toString();
	}

	@Override
	public int hashCode() {
		if ( this.hashCode == null ) {
			this.hashCode = 5*this.values.hashCode();
		}
		return this.hashCode;
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof SetTerm) ) {
    		return false;
    	}
    	SetTerm t = (SetTerm)o;

    	return this.values.equals(t.values); 
    }
	
	@Override
	public Iterator<Term> iterator() {
		return this.values.iterator();
	}
}
