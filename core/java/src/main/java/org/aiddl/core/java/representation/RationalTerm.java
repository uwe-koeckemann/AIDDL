package org.aiddl.core.java.representation;

import java.math.BigInteger;

import org.aiddl.core.java.container.Container;


/** A term representing a rational number.
 * @author Uwe Koeckemann
 *
 */
public class RationalTerm extends NumericalTerm {
	private Long p;
	private Long q;
	
	protected RationalTerm( Long p, Long q ) {
		if ( q != 0 ) {
			this.p = p;
			this.q = q;
		} else {
			throw new IllegalArgumentException("Rational number requires non-zero denominator q.");
		}
	}
	
	protected RationalTerm( Integer p, Integer q ) {
		if ( q != 0 ) {
			this.p = p.longValue();
			this.q = q.longValue();
		} else {
			throw new IllegalArgumentException("Rational number requires non-zero denominator q.");
		}
	}
	
	@Override
	public Long getNumerator() {
		return p;
	}
	
	@Override
	public Long getDenominator() {
		return q;
	}	
	
	@Override
	public Double getDoubleValue() {
		return Double.valueOf(this.p) / Double.valueOf(this.q); 
	}
	
	@Override
	public NumericalTerm add( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.add(this);
		}
		if ( x.isNaN() ) return x;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return new RationalTerm(this.p * x.getDenominator() + x.getNumerator() * this.q, this.q * x.getDenominator()).shorten();
		}
		return Term.real( x.getDoubleValue() + this.getDoubleValue() );
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
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return new RationalTerm(this.p * x.getDenominator() - x.getNumerator() * this.q, this.q * x.getDenominator()).shorten();
		}
		return Term.real( x.getDoubleValue() - this.getDoubleValue() );
	}
	@Override
	public NumericalTerm mult( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.mult(this);
		}
		if ( x.isNaN() ) return x;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return new RationalTerm(this.p*x.getNumerator(), this.q*x.getDenominator()).shorten();
		}
		return Term.real( x.getDoubleValue() * this.getDoubleValue() );
	}
	@Override
	public NumericalTerm div( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return new RationalTerm(0,1);
		}
		if ( x.isNaN() || x.isZero() ) return Term.nan();
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return new RationalTerm(this.p * x.getDenominator(), this.q * x.getNumerator()).shorten();
		}
		return Term.real( x.getDoubleValue() / this.getDoubleValue() );
	}
	@Override
	public boolean lessThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThan(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return this.p*x.getDenominator() < x.getNumerator()*this.q;
		}
		return this.getDoubleValue() < x.getDoubleValue();
	}
	@Override
	public boolean lessThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.greaterThanEq(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return this.p*x.getDenominator() <= x.getNumerator()*this.q;
		}
		return this.getDoubleValue() <= x.getDoubleValue();
	}
	@Override
	public boolean greaterThan( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThan(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return this.p*x.getDenominator() > x.getNumerator()*this.q;
		}
		return this.getDoubleValue() > x.getDoubleValue();
	}
	@Override
	public boolean greaterThanEq( NumericalTerm x ) {
		if ( x instanceof InfinityTerm ) {
			return x.lessThanEq(this);
		}
		if ( x.isNaN() ) return false;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm ) {
			return this.p*x.getDenominator() >= x.getNumerator()*this.q;
		}
		return this.getDoubleValue() >= x.getDoubleValue();
	}
	private RationalTerm shorten() {
		BigInteger b1 = BigInteger.valueOf(p);
		BigInteger b2 = BigInteger.valueOf(q);
		BigInteger gcd = b1.gcd(b2);
		return Term.rational(p/gcd.longValue(), q/gcd.longValue());
	}
	@Override
	public boolean equalTo( NumericalTerm x ) {
		if ( x instanceof InfinityTerm )
			return false;
		if ( x.isNaN() ) return false;
		if ( x instanceof RationalTerm || x instanceof IntegerTerm )
			return this.p*x.getDenominator() == x.getNumerator()*this.q;
		return this.getDoubleValue() == x.getDoubleValue();
	}
	@Override
	public boolean isZero() {
		return this.p.equals(0L);
	}
	@Override
	public boolean isPositive() {
		return this.p > 0L;
	}
	@Override
	public boolean isNegative() {
		return this.p < 0L;
	}
	
	@Override
	public RationalTerm asRational() {
		return this;
	}
	
	@Override
	public RationalTerm substitute(Substitution s) {
		return this;
	}
	
	@Override
	public RationalTerm resolve(Container db) {
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
		return p.toString() + "/" + q.toString();
	}

	@Override
	public int hashCode() {
		return p.hashCode() + 3*q.hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof RationalTerm) ) {
    		return false;
    	}
    	RationalTerm t = (RationalTerm)o;
    	return this.p*t.q == t.p*this.q; 
    }
}
