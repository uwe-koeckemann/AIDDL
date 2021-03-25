package org.aiddl.core.representation;

/** A term representing a boolean value.
 * @author Uwe Köckemann
 *
 */
public class BooleanTerm extends SymbolicTerm {
	Boolean value;
	
	/**
	 * Static term for the boolean value true
	 */
	public static BooleanTerm TRUE = new BooleanTerm(true);
	/**
	 * Static term for the boolean value false
	 */
	public static BooleanTerm FALSE = new BooleanTerm(false);
	
	private BooleanTerm( Boolean value ) {
		super(value.toString());
		this.value = value;
	}
	
	/** Negate this term.
	 * @return negated boolean term
	 */
	public BooleanTerm not( ) {
		return this.value ? FALSE : TRUE;
	}
	/** Apply logical and to this and another boolean term.
	 * @param b second boolean
	 * @return <code>this && b</code>
	 */
	public BooleanTerm and( BooleanTerm b ) {
		return this.value && b.value ? TRUE : FALSE;
	}
	/** Apply logical or to this and another boolean term.
	 * @param b second boolean
	 * @return <code>this || b</code>
	 */
	public BooleanTerm or( BooleanTerm b ) {
		return this.value || b.value ? TRUE : FALSE;
	}
	/** Apply logical xor to this and another boolean term.
	 * @param b second boolean
	 * @return  <code>this ^ b</code>
	 */
	public BooleanTerm xor( BooleanTerm b ) {
		return this.value ^ b.value ? TRUE : FALSE;
	}
	
	@Override
	public Boolean getBooleanValue() {
		return value;
	}
	
	@Override
	public boolean equals( Object o ) {
		if ( o instanceof BooleanTerm ) {
			return this.value.equals(((BooleanTerm)o).value);
		}
		return false;
	}
}
