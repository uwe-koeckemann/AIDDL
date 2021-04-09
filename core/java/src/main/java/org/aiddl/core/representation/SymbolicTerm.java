package org.aiddl.core.representation;

import org.aiddl.core.container.Container;
import org.aiddl.core.tools.LockableList;

/** Term representing a symbol.
 * @author Uwe Koeckemann
 *
 */
public class SymbolicTerm extends Term {
	String value;
	
	protected SymbolicTerm( String value ) {
		this.value = value;
	}
	
	public SymbolicTerm concat( SymbolicTerm s ) {
		return Term.sym(this.value + "." + s.value);
	}
	
	public ListTerm split() {
		LockableList L = new LockableList();
		for ( String part : value.split("\\.") ) {
			L.add(Term.sym(part));
		}
		return Term.list(L);
	}
	
	@Override 
	public SymbolicTerm asSym() {
		return this;
	}
	
	@Override
	public String getStringValue() {
		return value;
	}
	
	@Override
	public Substitution match(Term t) {
		if ( this.equals(t) ) {
			return new Substitution();
		}
		return super.match(t);
	}
	
	@Override
	public Term substitute(Substitution s) {
		return s.substitute(this);
	}
	
	@Override
	public SymbolicTerm resolve(Container db) {
		return this;
	}
	
	@Override
	public boolean isGround() {
		return true;
	}
	
	@Override
	public boolean isComplex() {
		return false;
	}

	@Override
	public boolean isVariable() {
		return false;
	}

	@Override
	public boolean isConstant() {
		return true;
	}

	@Override
	public String toString() {
		return value.toString();
	}

	@Override
	public int hashCode() {
		return value.hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
    	return this == o;
//    	if ( this == o ) {
//    		return true;
//    	}
//    	if ( !(o instanceof SymbolicTerm) ) {
//    		return false;
//    	}
//    	SymbolicTerm t = (SymbolicTerm)o;
//    	return this.value.equals(t.value); 
    }
}
