package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/**
 * Term with a string value.
 * 
 * @author Uwe Köckemann
 *
 */
public class StringTerm extends Term {
	String value;
	
	protected StringTerm( String value ) {
		this.value = value;
		if ( !this.value.startsWith("\"") ) {
			this.value = "\"" + this.value;
		}
		if ( !this.value.endsWith("\"") ) {
			this.value = this.value + "\"";
		}
	}
	
	@Override
	public String getStringValue() {
		return value.substring(1, value.length()-1);
	}
	
	@Override
	public Substitution match(Term t) {
		if ( this.equals(t) ) {
			return new Substitution();
		}
		return super.match(t);
	}
	
	@Override
	public StringTerm substitute(Substitution s) {
		return this;
	}
	
	@Override
	public StringTerm resolve(Container db) {
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
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof StringTerm) ) {
    		return false;
    	}
    	StringTerm t = (StringTerm)o;
    	return this.value.equals(t.value); 
    }
}
