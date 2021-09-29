package org.aiddl.core.representation;

/** Super class for numerical terms. Provides various methods only applicable to numerical terms. 
 * @author Uwe Koeckemann
 *
 */
public abstract class NumericalTerm extends Term implements Comparable<NumericalTerm> {	
		
	/** Test if the numerical value of this term is zero.
	 * @return <code>true</code> if this term has the value 0.
	 */
	public abstract boolean isZero();
	/** Test if the numerical value of this term is positive.
	 * @return <code>true</code> if this term has a value greater than zero
	 */
	public abstract boolean isPositive();
	/** Test if the numerical value of this term is negative.
	 * @return <code>true</code> if this term has a value smaller than zero
	 */
	public abstract boolean isNegative();
	
	/** Add another numerical term to this one. 
	 * @param v a numerical term
	 * @return sum of both terms
	 */
	public abstract NumericalTerm add ( NumericalTerm v );
	/** Subtract another numerical term from this one. 
	 * @param v a numerical term
	 * @return result of subtraction
	 */
	public abstract NumericalTerm sub ( NumericalTerm v );
	/** Multiply another numerical term with this one.
	 * @param v a numerical term
	 * @return product of both terms
	 */
	public abstract NumericalTerm mult( NumericalTerm v );
	/** Divide this term by another numerical term. 
	 * @param v a numerical term
	 * @return result of the division
	 */
	public abstract NumericalTerm div ( NumericalTerm v );
		
	/** Check if this numerical term less than another
	 * @param v a numerical term
	 * @return this < v
	 */
	public abstract boolean lessThan( NumericalTerm v );
	/** Check if this numerical term less than or equal to another
	 * @param v a numerical term
	 * @return this <= v
	 */
	public abstract boolean lessThanEq( NumericalTerm v );
	/** Check if this numerical term greater than another
	 * @param v a numerical term
	 * @return this > v
	 */
	public abstract boolean greaterThan( NumericalTerm v );
	/** Check if this numerical term greater than or equal to another
	 * @param v a numerical term
	 * @return this >= v
	 */
	public abstract boolean greaterThanEq( NumericalTerm v );
	/** Check if this numerical term is equal to another
	 * @param v a numerical term
	 * @return this == v
	 */
	public abstract boolean equalTo( NumericalTerm v );

	public boolean isNaN() { return false; }
	
	public NumericalTerm min( NumericalTerm v ) {
		if ( this.greaterThanEq(v) ) {
			return v;
		}
		return this;
	}
	
	public NumericalTerm max( NumericalTerm v ) {
		if ( this.greaterThanEq(v) ) {
			return this;
		}
		return v;
	}
	
	@Override 
	public NumericalTerm asNum()  {
		return this;
	}
	
	@Override
	public int compareTo(NumericalTerm x) {
		if ( this.isNaN() || x.isNaN() )
			throw new IllegalArgumentException("Cannot compare NaN with this method (no consistent result possible)");
		// Using subtraction would lead to problems with InfinityTerm
		if ( this.lessThan(x) ) {
			return -1;
		} 
		if ( this.equalTo(x) ) {
			return 0;
		}
		return 1;
	}
}
