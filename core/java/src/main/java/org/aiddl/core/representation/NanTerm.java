package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/** Term representing an infinite numerical value.
 * @author Uwe Koeckemann
 *
 */
public class NanTerm extends NumericalTerm {
	protected static NanTerm NaN = new NanTerm();
	protected NanTerm() {}
		
	@Override
	public NumericalTerm add( NumericalTerm x ) {
		return this;
	}
	@Override
	public NumericalTerm sub( NumericalTerm x ) {
		return this;
	}
	@Override
	public NumericalTerm mult( NumericalTerm x ) {
		return this;
	}
	@Override
	public NumericalTerm div( NumericalTerm x ) {
		return this;
	}
		
	@Override
	public boolean lessThan( NumericalTerm x ) {
		return false;
	}
	@Override
	public boolean lessThanEq( NumericalTerm x ) {
		return false;
	}
	@Override
	public boolean greaterThan( NumericalTerm x ) {
		return false;
	}
	@Override
	public boolean greaterThanEq( NumericalTerm x ) {
		return false;
	}
	
	@Override
	public boolean equalTo( NumericalTerm t ) {
		return false;
	}

	@Override
	public boolean isNaN() { return true; }
	
	@Override
	public boolean isZero() {
		return false;
	}
	@Override
	public boolean isPositive() {
		return false;
	}
	@Override
	public boolean isNegative() {
		return false;
	}
	
	@Override
	public NumericalTerm substitute(Substitution s) {
		return this;
	}
	
	@Override
	public NanTerm resolve(Container db) {
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
		return "NaN";
	}

	@Override
	public int hashCode() {
		return 23*this.toString().hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
    	return false;
    }
}
