package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/** Term representing an infinite numerical value.
 * @author Uwe Koeckemann
 *
 */
public class InfinityTerm extends NumericalTerm {
	private boolean isNegative;
	
	protected static final InfinityTerm Positive = new InfinityTerm(false);
	protected static final InfinityTerm Negative = new InfinityTerm(true);
	
	private InfinityTerm( boolean isNegative ) {
		this.isNegative = isNegative;
	}
		
	@Override
	public NumericalTerm add( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			InfinityTerm inf = (InfinityTerm)x;
			if ( this.isNegative != inf.isNegative ) {
				return Term.nan();
			}
		}
		if ( x.isNaN() ) return x;
		return this;
	}
	@Override
	public NumericalTerm sub( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			InfinityTerm inf = (InfinityTerm)x;
			if ( this.isNegative == inf.isNegative ) {
				return Term.nan(); // throw new IllegalArgumentException("Result of subtracting " + x + " from " + this + " not defined.");
			}
		}
		return this;
	}
	@Override
	public NumericalTerm mult( NumericalTerm x ) {
		if ( x.isZero() ) return Term.nan();
		else if ( this.isPositive() && x.isPositive() ) {
			return this;
		} else if ( this.isNegative() && x.isNegative() ) {
			return new InfinityTerm(false);
		} else {
			return new InfinityTerm(true);
		}
			
	}
	@Override
	public NumericalTerm div( NumericalTerm x ) {
		if ( x instanceof InfinityTerm )
			return Term.nan(); // throw new IllegalArgumentException("Trying to divide " + this + " by " + x);
		if ( x.isZero() ) return Term.nan(); // throw new IllegalArgumentException("Trying to divide " + this + " by " + x);
		else if ( this.isPositive() && x.isPositive() ) {
			return this;
		} else if ( this.isNegative() && x.isNegative() ) {
			return new InfinityTerm(false);
		} else {
			return new InfinityTerm(true);
		}
	}
		
	@Override
	public boolean lessThan( NumericalTerm x ) {
		if ( x.isNaN() ) return false;
		if ( this.isNegative() && !x.equals(new InfinityTerm(true)) ) {
			return true;
		} else {
			return false;
		}
	}
	@Override
	public boolean lessThanEq( NumericalTerm x ) {
		if ( x.isNaN() ) return false;
		if ( this.isNegative() ) {
			return true;
		} else {
			return false;
		}
	}
	@Override
	public boolean greaterThan( NumericalTerm x ) {
		if ( x.isNaN() ) return false;
		if ( this.isPositive() && !x.equals(new InfinityTerm(false)) ) {
			return true;
		} else {
			return false;
		}
	}
	@Override
	public boolean greaterThanEq( NumericalTerm x ) {
		if ( x.isNaN() ) return false;
		if ( this.isPositive() ) {
			return true;
		} else {
			return false;
		}
	}
	
	@Override
	public boolean equalTo( NumericalTerm t ) {
		return this.equals(t);
	}
	
	@Override
	public boolean isZero() {
		return false;
	}
	@Override
	public boolean isPositive() {
		return !this.isNegative;
	}
	@Override
	public boolean isNegative() {
		return this.isNegative;
	}
	@Override
	public boolean isInf() { return true; }
	@Override
	public boolean isInfPos() { return !this.isNegative; }
	@Override
	public boolean isInfNeg() { return this.isNegative; }
	
	@Override
	public NumericalTerm substitute(Substitution s) {
		return this;
	}
	
	@Override
	public InfinityTerm resolve(Container db) {
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
		if ( isNegative )
			return "-INF";
		else
			return "+INF";
	}

	@Override
	public int hashCode() {
		return 17*this.toString().hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
		/*if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof InfinityTerm) ) {
    		return false;
    	}
    	InfinityTerm t = (InfinityTerm)o;
    	return this.isNegative == t.isNegative;*/
		return false;
    }
}
