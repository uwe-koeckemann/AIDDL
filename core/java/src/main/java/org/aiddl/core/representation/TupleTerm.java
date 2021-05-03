package org.aiddl.core.representation;

import java.util.List;

import org.aiddl.core.container.Container;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableList.ListType;
import org.aiddl.core.tools.MapType;
import org.aiddl.core.tools.Profiler;
import org.aiddl.core.tools.StopWatch;

/** A term representing a tuple of terms.
 * @author Uwe Koeckemann
 *
 */
public class TupleTerm extends Term {
	LockableList values;
		
	Integer hashCode = null;
	
	protected TupleTerm( Term... values ) {
		this.values = new LockableList(ListType.ArrayList, MapType.LinkedHashMap);
		for ( int i = 0 ; i < values.length ; i++ ) {
			this.values.add(values[i]);
		}
		this.values.lock();
	}
	
	protected TupleTerm( List<? extends Term> values ) {
		this.values = new LockableList(ListType.ArrayList, MapType.LinkedHashMap);
		for ( Term t : values ) {
			this.values.add(t);
		}
		this.values.lock();
	}
	
	protected TupleTerm( LockableList values ) {
		this.values = values;
		this.values.lock();
	}
	
	@Override
	public Integer size() {
		return this.values.size();
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
	public Term get( int i ) {
		return this.values.get(i);
	}
	
	public TupleTerm put( Term key, Term value ) {
		LockableList copy = new LockableList(this.values.getListType(), this.values.getMapType()); 
		for ( Term e : this.values ) {
			if ( !(e instanceof KeyValueTerm) || !e.getKey().equals(key) ) { 
				copy.add(e);
			}
		}
		copy.put(key,  value);
		return Term.tuple(copy);
	}
	
	public boolean containsKey( Term key ) {
		return this.values.containsKey(key);
	}
	
	/** 
	 * Check if the map contained in this collection is unique 
	 * @return <code>true</code> if this collection assigns a unique value to each key, <code>false</code> otherwise
	 */
	public boolean isUniqueMap() {
		return this.values.isUniqueMap();
	}
	
	@Override
	public TupleTerm asTuple() {
		return this;
	}	
	
	@Override
	public ListTerm asList() {
		return Term.list(this.values);
	}
	
	@Override
	public Substitution match(Term t) {
		if ( t instanceof TupleTerm ) {
			Substitution sCombined = new Substitution();
			TupleTerm tList = (TupleTerm)t;
			
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
	public TupleTerm substitute(Substitution s) {
		LockableList s_new = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.substitute(s));
		}		
		return Term.tuple(s_new);
	}
	
	@Override
	public TupleTerm resolve(Container db) {
		LockableList s_new = new LockableList(this.values.getListType(), this.values.getMapType());
		for ( Term v : this.values ) {
			s_new.add(v.resolve(db));
		}		
		return Term.tuple(s_new);
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
		sB.append("(");
		if ( this.values.size() > 0 ) 
			sB.append(values.get(0));
		for ( int i = 1 ; i < values.size() ; i++ ) {
			sB.append(" ");
			sB.append(values.get(i).toString());
		}
		sB.append(")");
		return sB.toString();
	}

	@Override
	public int hashCode() {
		if ( this.hashCode == null ) {
			this.hashCode = 3*this.values.hashCode();
		}
		return this.hashCode;
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( (!(o instanceof TupleTerm)) || this.hashCode() != o.hashCode() ) {
    		return false;
    	}
    	TupleTerm t = (TupleTerm)o;
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
}
