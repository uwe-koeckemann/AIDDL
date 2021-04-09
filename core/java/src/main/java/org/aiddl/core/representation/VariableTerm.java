package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/** Term representing a variable
 * @author Uwe Koeckemann
 *
 */
public class VariableTerm extends Term {
	private final String exportValue;
	private final String value;
	private static int anonymousVariableCount = 0;
	private final boolean isAnonymous;
		
	protected VariableTerm( ) {
		this.value = "?_X" + ++anonymousVariableCount;
		this.exportValue = "_";
		isAnonymous = true;
	}
	
	protected VariableTerm( String value ) {
		if ( !value.startsWith("?")) {
			this.value = "?" + value;
		} else {
			this.value = value;
		}
		this.exportValue = this.value;
		isAnonymous = false;	
	}
	
	protected VariableTerm( String value, String moduleName ) {
		if ( !value.startsWith("?")) {
			this.value = "?" + value; //+ moduleName + "." + value;
			this.exportValue = "?" + value;
		} else {
			this.value = "?" + value.substring(1); // moduleName + "." + value.substring(1);
			this.exportValue = value;
		}
		isAnonymous = false;	
	}
	
	@Override
	public Substitution match(Term t) {
		Substitution s = new Substitution();
		s.add(this, t);
		return s;
	}
	
	@Override
	public Term substitute(Substitution theta) { 
		Term newTerm = theta.substitute(this);
		if ( newTerm.equals(this)) {
			return this;
		} else {
			return newTerm; //.substitute(theta);
		}
	}
	
	@Override
	public VariableTerm resolve(Container db) {
		return this;
	}
	
	@Override
	public boolean isGround() {
		return false;
	}
	
	@Override
	public boolean isComplex() {
		return false;
	}

	@Override
	public boolean isVariable() {
		return true;
	}

	@Override
	public boolean isConstant() {
		return false;
	}

	@Override
	public String toString() {
		return this.exportValue;
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
    	if ( !(o instanceof VariableTerm) ) {
    		return false;
    	}
    	VariableTerm t = (VariableTerm)o;
    	return this.value.equals(t.value); 
    }
}
