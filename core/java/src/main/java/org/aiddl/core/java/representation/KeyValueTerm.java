package org.aiddl.core.java.representation;

import org.aiddl.core.java.container.Container;

/** Term representing a key and a value
 * @author Uwe Koeckemann
 *
 */
public class KeyValueTerm extends Term {
	Term key;
	Term value;
	
	protected KeyValueTerm( Term from, Term to ) {
		if ( from == null || to == null ) {
			throw new IllegalArgumentException("Null argument not allowed for key value term: " + from + ":" + to);
		}
		this.key = from;
		this.value = to;
	}
	
	@Override
	public Term getKey() {
		return this.key;
	}
	@Override
	public Term getValue() {
		return this.value;
	}
		
	@Override
	public Term get( Term t ) {
		return this.key.equals(t) ? value : null;
	}
	
	@Override
	public Term getOrDefault( Term key, Term defaultValue ) {
		return this.key.equals(key) ? value : defaultValue;
	}

	@Override
	public Substitution match(Term t) {
		if ( t instanceof KeyValueTerm ) {
			Substitution cSub = this.getKey().match(t.getKey());
			if ( cSub == null || !cSub.add(this.getValue().match(t.getValue())) ) {
				return null;
			}
			return cSub;
		} 
		return super.match(t);
	}
	
	@Override
	public KeyValueTerm substitute(Substitution s) {
		return Term.keyVal(this.getKey().substitute(s), this.getValue().substitute(s));
	}
	
	@Override
	public KeyValueTerm resolve( Container db ) {
		return Term.keyVal(this.getKey().resolve(db), this.getValue().resolve(db));
	}

	@Override
	public boolean isGround() {
		return this.getKey().isGround() && this.getValue().isGround();
	}
	
	@Override
	public boolean isComplex() {
		return true;
	}

	@Override
	public boolean isVariable() {
		return false;
	}

	@Override
	public boolean isConstant() {
		return false;
	}
	
	@Override
	public String toString() {
		StringBuilder sB = new StringBuilder();
		sB.append(this.getKey());
		sB.append(":");
		sB.append(this.getValue());
		return sB.toString();
	}

	@Override
	public int hashCode() {
		return key.hashCode() + 3*value.hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof KeyValueTerm) ) {
    		return false;
    	}
    	Term t = (Term)o;
    	return this.getKey().equals(t.getKey()) && this.getValue().equals(t.getValue()); 
    }
}
