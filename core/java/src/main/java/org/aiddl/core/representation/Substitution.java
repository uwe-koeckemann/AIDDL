package org.aiddl.core.representation;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.tools.LockableSet;

/** A substitution is a map from variable terms to terms that can 
 * be applied to recursively replace variables in terms.
 * @author Uwe Köckemann
 *
 */
public class Substitution {
	private boolean forced = false;
	
	private Map<Term,Term> map = new HashMap<Term,Term>();
	
	/**
	 * Creates an empty substitution.
	 */
	public Substitution() {	}
	
	/**
	 * Creates an empty substitution.
	 */
	public Substitution( boolean forced ) {
		this.forced = forced;
	}
	
	/**
	 * Creates a substitution from a collection term.
	 */
	public Substitution( Term C ) {	
		for ( Term kvp : C.asCollection() ) {
			if ( kvp instanceof KeyValueTerm ) {
				this.add(kvp.getKey(), kvp.getValue());
			}
		}
	}
	
	
	/**
	 * Check if this is a forced substitution (i.e. a substitution that will replace terms inside function references)
	 * @return
	 */
	public boolean isForced() { return this.forced; }
	
	/**
	 * Add a new substitution to mapping providing two {@link Term} for <i>from</i> and <i>to</i>.
	 * Only adds if no loop of substitutions is created.
	 * Overwrites any existing substitution of <i>from</i>.
	 * @param from {@link Term} that will be replaced 
	 * @param to {@link Term} that will be substituted
	 */
	public void add(Term from, Term to) {
		if ( from == null || to == null ) {
			throw new IllegalArgumentException("Cannot add null arguments: " + from + "/" + to);
		} else if ( from.equals(to) ) {
			return;
		} 
//		else if ( isSelfReferentialSub(from, to) ) {
//			throw new IllegalArgumentException("Self-referential substitution: from " + from + " to " + to);
//		}
		if (!addingWouldCauseLoop(from,to)) {
			map.put(from, to);
		}
	}
	
	/**
	 * Add another {@link Substitution} <i>s</i> to this one, if the two are compatible.
	 * Two {@link Substitution}s are incompatible if they try to substitute the same {@link Term}
	 * with two different {@link Term}s.
	 * @param s The {@link Substitution} to add.
	 * @return <i>true</i> if {@link Substitution} <i>s</i> was compatible and successfully added, <i>false</i> otherwise.
	 */
	public boolean add(Substitution s) {
		if ( s == null ) {
			return false;
		}
			
		for ( Term from : s.map.keySet() ) {
			
			if ( addingWouldCauseLoop(from, s.getMap().get(from)) ) {
				return false;
			}
						
			if ( ! this.compatibleWithAll(from, s.map.get(from)) ) {
				return false;
			}
		} 
		
		for ( Term from : s.map.keySet() ) {
			Term to = s.map.get(from);
			if ( !from.equals(to)) {
				map.put(from, s.map.get(from));
			}
		}
				
		return true;
	}
	
	/**
	 * Checks if a <i>from</i> and <i>to</i> {@link Term} can be added to this {@link Substitution} without 
	 * conflicting with any of its existing substitutions. 
	 * @param from {@link Term} that should be replaced by <i>to</i>.
	 * @param to {@link Term} that <i>from</i> is to be replaced with.
	 * @return <i>true</i> if new substitution can be added without conflict, <i>false</i> otherwise.
	 */
	private boolean compatibleWithAll(Term from, Term to) {
		for ( Term fromThis : map.keySet() ) {
			if ( ! this.compatibleWithOne(fromThis,map.get(fromThis),from,to) ) {
				return false;
			}
		}		
		return true;
	}
	
	/**
	 * Checks if a substitution <i>f1/t1</i> is compatible with another substitution <i>f2/t2</i>.
	 * @param f1
	 * @param t1
	 * @param f2
	 * @param t2
	 * @return <i>true</i> if <i>f1</i> != <i>f2</i> or (<i>f1</i> == <i>f2</i> and <i>t1</i> == <i>t2</i> and <i>t1,t2</i> are constants), <i>false<i> otherwise. 
	 */
	private boolean compatibleWithOne(Term f1, Term t1, Term f2, Term t2) {
		if ( f1.equals(f2) ) {
			if ( t1.isConstant() && t2.isConstant() ) {
				if ( ! t1.equals(t2)) {
					return false;
				}
			} else if ( t1.isVariable() && t2.isVariable() ) {
				if ( !t1.equals(t2) ) {
					return false;
				}
			}
		}
		return true;
	}
	
	/**
	 * Get the map of this substitution
	 * @return the map
	 */
	public Map<Term,Term> getMap() {
		return map;
	}
	
	/**
	 * Check if a substitution would create a circle (e.g., adding X/Y to {Y/X} would do so).
	 * @param from {@link Term} that will be replaced 
	 * @param to {@link Term} that will be substituted
	 * @return <code>true</code> if adding from/to to this substitution would cause a loop
	 */
	public boolean addingWouldCauseLoop(Term from, Term to) {
		Set<Term> cloud = new HashSet<Term>();
		cloud.add(to);
		
		int sizeBefore = -1;
		while ( sizeBefore != cloud.size() ) {
			
			sizeBefore = cloud.size();
			
			Set<Term> addList = new HashSet<Term>();
			for ( Term t : cloud ) {
				addList.add(map.get(t));
			}
			cloud.addAll(addList);
			
			if ( cloud.contains(from) ) {	
				/* 
				 * Path between to and from exists,
				 * so new substitution would create a loop. 
				 */
				return true;
			}
		}
		return false;
	}
	
	public static boolean isSelfReferentialSub( Term from, Term to ) {
		Substitution s = new Substitution();
		s.map.put(from, Term.sym("--NIL--"));
		
		return !to.substitute(s).equals(to); 
	}
	
	/**
	 * Return inverse of this substitution. 
	 * @throws IllegalArgumentException when substitution does not have a unique inverse
	 * @return substitution that maps this substitution's values to their keys if possible, <code>null</code> otherwise
	 */
	public Substitution inverse() {
		Substitution inv = new Substitution();
		for ( Term k : this.map.keySet() ) {
			inv.map.putIfAbsent(this.map.get(k), k);
//			if ( null != inv.map.putIfAbsent(this.map.get(k), k) ) {
//				return null;
//			}
			
		}
		return inv;
	}
	
	/**
	 * Apply this {@link Substitution} to {@link Term} <i>t</i>.
	 * @param from The {@link Term} to substitute
	 * @return new {@link Term} if substitute for <i>t</i> exists, <i>t</i> otherwise.
	 */
	public Term substitute( Term from ) {
		return map.getOrDefault(from, from);
	}
	
	/**
	 * Convert substitution to a term.
	 * @return set term representation of substitution
	 */
	public Term getTerm() {
		LockableSet S = new LockableSet();
		S.putAll(map);
		return Term.set(S);
	}

	/** Create a copy of this substitution.
	 * @return a new substitution that can be changed without changing this one
	 */
	public Substitution copy() {
		Substitution c = new Substitution();
		c.add(this);
		return c;
	}
	
	@Override
	public String toString() {
		StringBuilder sB = new StringBuilder();
		sB.append("{");
		int i = 0;
		for ( Term k : this.map.keySet() ) {
			sB.append(k);
			sB.append(":");
			sB.append(this.map.get(k));
			if ( i < this.map.size()-1 ) {
				sB.append(" ");
			}
			i++;
		}
		sB.append("}");
		return sB.toString();
	}

	@Override
	public int hashCode() {
		return 3 * this.map.hashCode();
	}

	@Override
	public boolean equals(Object o) {
		if ( o instanceof Substitution ) {
			Substitution s = (Substitution)o;
			return this.map.equals(s.map);
		}
		return false;
	}
	
}
