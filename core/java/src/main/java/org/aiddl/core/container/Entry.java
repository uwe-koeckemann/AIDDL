package org.aiddl.core.container;

import java.util.function.Function;

import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Combine type, name, and value. Every AIDDL file uses entries as the basic building block.
 * 
 * @author Uwe Koeeckemann
 *
 */
public class Entry	 {
	Term type;
	Term name;
	Term content;
	
	/** Create an entry.
	 * @param type type of the entry
	 * @param name name of the entry
	 * @param value value of the entry
	 */
	public Entry(Term type, Term name, Term value) {
		
		if ( type == null || name == null || value == null ) 
			throw new IllegalArgumentException("Entry constructor needs non-null arguments.");
		
		this.type = type;
		this.name = name;
		this.content = value;
	}
	
	/** Get the type of this entry.
	 * @return term representing the type
	 */
	public Term getType() {
		return type;
	}
	
	/** Get the name of this entry.
	 * @return term representing the name
	 */
	public Term getName() {
		return name;
	}
	
	/** Get the value of this entry.
	 * @return term representing the value
	 */
	public Term getValue() {
		return content;
	}
	
	/**
	 * Convert entry to a tuple term.
	 * @return tuple of form (type, name, value)
	 */
	public TupleTerm asTuple() {
		return Term.tuple(type, name, content);
	}
	
	/** Create s new entry by applying substitution to all its elements. 
	 * @param s substitution
	 * @return entry after applying substitution
	 */
	public Entry substitute ( Substitution s ) {
		return new Entry(	this.type.substitute(s),
							this.name, //.substitute(s),
							this.content.substitute(s) );
	}

	@Override
	public int hashCode() {
		return this.type.hashCode() + 3*this.name.hashCode() + 7*this.content.hashCode();
	}
	
	@Override
	public boolean equals( Object o ) {
		if ( o instanceof Entry ) {
			Entry e = (Entry)o;
			return e.type.equals(this.type) && e.name.equals(this.name) && e.content.equals(this.content);
		}
		return false;
	}
	
	@Override
	public String toString() {
		StringBuilder sB = new StringBuilder();
		sB.append("(");
		sB.append(type.toString());
		sB.append(" ");
		sB.append(name.toString());
		sB.append(" ");
		sB.append(content.toString());
		sB.append(")");
		return sB.toString();		
	}
}
