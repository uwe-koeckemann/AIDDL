package org.aiddl.core.tools;

import java.util.Comparator;

import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Default comparator for terms. Considers numerical and defaults to string comparison.
 * TODO: Add special case for tuples
 * 
 * @author Uwe Köckemann
 *
 */
public class TermComparator implements Comparator<Term> {

	@Override
	public int compare(Term o1, Term o2) {
		if ( o1 instanceof NumericalTerm && o2 instanceof NumericalTerm ) {
			return o1.asNum().compareTo(o2.asNum());
		} else if ( o1 instanceof TupleTerm && o2 instanceof TupleTerm && o1.size() == o2.size() ) {
			for ( int i = 0 ; i < o1.size() ; i++ ) {
				int comp = this.compare(o1.get(i), o2.get(i));
				if ( comp != 0 ) {
					return comp;
				}
			}
			return 0;
		}
		return o1.toString().compareTo(o2.toString());
	}

}
