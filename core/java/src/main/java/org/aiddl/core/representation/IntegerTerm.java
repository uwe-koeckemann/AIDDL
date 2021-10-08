package org.aiddl.core.representation;

import org.aiddl.core.container.Container;

/** Numerical term representing an integer value. 
 * @author Uwe Koeckemann
 *
 */
public class IntegerTerm extends NumericalTerm {
	private Long value;
	
	protected IntegerTerm( Long value ) {
		this.value = value;
	}
	
	@Override
	public Long getLongValue() {
		return value;
	}
	
	@Override
	public Integer getIntValue() {
		return value.intValue();
	}
	
	@Override
	public Long getNumerator() {
		return value;
	}
	
	@Override
	public Long getDenominator() {
		return 1L;
	}
	
	@Override
	public Double getDoubleValue() {
		return Double.valueOf(value);
	}
	
	@Override
	public NumericalTerm add( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.add(this);
		}
		if ( x.isNaN() ) {
			return x;
		}
		if ( x instanceof IntegerTerm )
			return new IntegerTerm(this.value + ((IntegerTerm)x).getLongValue());
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).add(x);
		return Term.real(Double.valueOf(this.value) + x.getDoubleValue() );		
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
		if ( x.isNaN() ) {
			return x;
		}
		if ( x instanceof IntegerTerm )
			return new IntegerTerm(this.value - ((IntegerTerm)x).getLongValue());
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).sub(x);
		return Term.real(Double.valueOf(this.value) - x.getDoubleValue() );		
	}
	@Override
	public NumericalTerm mult( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.mult(this);
		}
		if ( x.isNaN() ) {
			return x;
		}
		if ( x instanceof IntegerTerm )
			return new IntegerTerm(this.value * ((IntegerTerm)x).getLongValue());
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).mult(x);
		return Term.real(Double.valueOf(this.value) * x.getDoubleValue() );		
	}
	@Override
	public NumericalTerm div( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return Term.integer(0);
		}
		if ( x.isNaN() || x.isZero() ) {
			return Term.nan();
		}
		if ( x instanceof IntegerTerm )
			return new IntegerTerm(this.value / ((IntegerTerm)x).getLongValue());
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).div(x);
		return Term.real(Double.valueOf(this.value) / x.getDoubleValue() );		
	}

	/** Get the remainder of dividing this integer term by another integer term.
	 * @param x the divisor
	 * @return the modulo of this / x
	 */
	public IntegerTerm mod( IntegerTerm x ) {
		return new IntegerTerm(this.value % x.getLongValue());
	}
	@Override
	public boolean lessThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThan(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof IntegerTerm )
			return this.value < x.getLongValue();
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).lessThan(x);
		return Double.valueOf(this.value) < x.getDoubleValue();		
	}
	@Override
	public boolean lessThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThanEq(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof IntegerTerm )
			return this.value <= x.getLongValue();
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).lessThanEq(x);
		return Double.valueOf(this.value) <= x.getDoubleValue();		
	}
	@Override
	public boolean greaterThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThan(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof IntegerTerm )
			return this.value > x.getLongValue();
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).greaterThan(x);
		return Double.valueOf(this.value) > x.getDoubleValue();		
	}
	@Override
	public boolean greaterThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThanEq(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof IntegerTerm )
			return this.value >= x.getLongValue();
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).greaterThanEq(x);
		return Double.valueOf(this.value) >= x.getDoubleValue();		
	}
	@Override
	public boolean equalTo( NumericalTerm x) {
		if ( x instanceof InfinityTerm )
			return false;
		if ( x.isNaN() ) return false;
		if ( x instanceof IntegerTerm )
			return this.value.equals(x.getLongValue());
		if ( x instanceof RationalTerm ) 
			return Term.rational(this.value, 1L).equalTo(x);
		return Double.valueOf(this.value).equals(x.getDoubleValue());
	}

	@Override
	public boolean isZero() {
		return this.value.equals(0L);
	}
	@Override
	public boolean isPositive() {
		return this.value > 0L;
	}
	@Override
	public boolean isNegative() {
		return this.value < 0L;
	}
	
	@Override
	public IntegerTerm asInt() {
		return this;
	}
		
	@Override
	public Term substitute(Substitution s) {
		return s.substitute(this);
	}
	
	@Override
	public IntegerTerm resolve(Container db) {
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
    	if ( !(o instanceof IntegerTerm) ) {
    		return false;
    	}
    	IntegerTerm t = (IntegerTerm)o;
    	return this.value.equals(t.value); 
    }
}
