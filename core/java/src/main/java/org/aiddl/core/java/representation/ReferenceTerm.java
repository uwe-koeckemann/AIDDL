package org.aiddl.core.java.representation;

import org.aiddl.core.java.container.Container;

/** A term representing a reference to a another term.
 * @author Uwe Koeckemann
 *
 */
public class ReferenceTerm extends Term {
	Term refName;
	Term alias;
	Term modName;
	
	protected ReferenceTerm( Term refName, Term modName ) {
		this.refName = refName;
		this.modName = modName;
	}
	
	protected ReferenceTerm( Term refName, Term localName, Term modName ) {
		this.refName = refName;
		this.alias = localName;
		this.modName = modName;
	}
	
	/**
	 * Convert reference to URI form if referenced name is symbolic
	 * @return Symbolic URI in form module.name or <code>null</code> if name not symbolic
	 */
	public SymbolicTerm convert2uri() {
		if ( refName instanceof SymbolicTerm )
			return modName.asSym().concat(refName.asSym());
		return null;
	}
	
	@Override
	public ReferenceTerm asRef() {
		return this;
	}
	
	@Override
	public Term getRefTarget() {
		return refName;
	}
	
	@Override
	public Term getRefAlias() {
		return alias;
	}
	
	@Override
	public Term getRefModule() {
		return modName;
	}
		
	@Override
	public Substitution match(Term t) {
		if ( t instanceof ReferenceTerm ) {
			ReferenceTerm tRef = (ReferenceTerm)t;
			Substitution s = this.refName.match(tRef.refName);
			Substitution s_next = null;
			if ( alias != null ) {
				s_next = this.alias.match(tRef.alias);
				if ( s_next == null || !s.add(s_next) ) {
					return null;
				}
				s_next = this.modName.match(tRef.modName);
				if ( s_next == null || !s.add(s_next) ) {
					return null;
				}
				return s;
			}
		}
		return super.match(t);
	}
	
	@Override
	public Term substitute(Substitution s) {
		if ( alias == null ) 
			return Term.ref(this.refName.substitute(s), this.modName.substitute(s));
		else
			return Term.ref(this.refName.substitute(s), this.alias.substitute(s), this.modName.substitute(s));
	}
	
	@Override
	public Term resolve(Container db) {
		Term resolved = db.resolveReference(this);
		if ( resolved == null ) {
			return this;
		} else if ( resolved.equals(this) ) {
			return this; 
		}
		return db.resolveReference(this).resolve(db);
	}
		
	@Override
	public boolean isGround() {
		return refName.isGround();
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
		if ( this.alias == null )
			return refName.toString() + "@" + modName.toString();
		return String.format("%s@%s", this.refName, this.alias);
	}

	@Override
	public int hashCode() {
		return 3 * refName.hashCode() + 7*this.modName.hashCode() +
				( this.alias == null ? 0 : 5*this.alias.hashCode() );
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof ReferenceTerm ) ) {
    		return false;
    	}
    	ReferenceTerm t = (ReferenceTerm)o;
    	if ( !this.refName.equals(t.refName) || !this.modName.equals(t.modName) ) {
    		return false;
    	}
    	if ( this.alias != null ) {
    		return this.alias.equals(t.alias);
    	} else {
    		if ( t.alias == null )
    			return true;
    		else
    			return false;
    	}
    }
}
