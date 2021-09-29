package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/** A term representing a real number.
 * @author Uwe Koeckemann
 *
 */
public class RealTerm extends NumericalTerm {
	private Double value;
	
	protected RealTerm( Double value ) {
		this.value = value;
	}
	
	@Override
	public Double getDoubleValue() {
		return value;
	}
	
	@Override
	public NumericalTerm add( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.add(this);
		}
		if ( x.isNaN() ) return x;
		return Term.real(this.value + x.getDoubleValue());
	}
	@Override
	public NumericalTerm sub( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			if ( x.isPositive() ) {
				return Term.infNeg();
			} else {
				return Term.infPos();
			}
		}
		if ( x.isNaN() ) return x;
		return Term.real(this.value - x.getDoubleValue());
	}
	@Override
	public NumericalTerm mult( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.mult(this);
		}
		if ( x.isNaN() ) return x;
		return Term.real(this.value * x.getDoubleValue());
	}
	@Override
	public NumericalTerm div( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return Term.rational(0L,1L);
		}
		if ( x.isNaN() || x.isZero() ) return Term.nan();
		return Term.real(this.value / x.getDoubleValue());
	}
	@Override
	public boolean lessThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThan(this);
		}
		if ( x.isNaN() ) return false;
		return this.value < x.getDoubleValue();
	}
	@Override
	public boolean lessThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThanEq(this);
		}
		if ( x.isNaN() ) return false;
		return this.value <= x.getDoubleValue();
	}
	@Override
	public boolean greaterThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThan(this);
		}
		if ( x.isNaN() ) return false;
		return this.value > x.getDoubleValue();
	}
	@Override
	public boolean greaterThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThanEq(this);
		}
		if ( x.isNaN() ) return false;
		return this.value >= x.getDoubleValue();
	}
	@Override
	public boolean equalTo( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return false;
		}
		if ( x.isNaN() ) return false;
		return this.value.equals(x.getDoubleValue());
	}
	@Override
	public boolean isZero() {
		return this.value.equals(Double.valueOf(0.0));
	}
	@Override
	public boolean isPositive() {
		return this.value > 0.0;
	}
	@Override
	public boolean isNegative() {
		return this.value < 0.0;
	}
	
	@Override
	public RealTerm asReal() {
		return this;
	}
		
	@Override
	public RealTerm substitute(Substitution s) {
		return this;
	}
	
	@Override
	public RealTerm resolve(Container db) {
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
    	if ( !(o instanceof RealTerm) ) {
    		return false;
    	}
    	RealTerm t = (RealTerm)o;
    	return this.value.equals(t.value); 
    }
}
