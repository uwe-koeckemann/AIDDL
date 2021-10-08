package org.aiddl.common.reasoning.temporal.allen_constraints;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class IntersectionChecker implements Function {

	@Override
	public Term apply(Term args) {
		CollectionTerm intervals = args.get(0).asCollection();
		Term intervalDomain = args.get(1);
		
		NumericalTerm st = Term.infNeg();
		NumericalTerm et = Term.infPos();
		
		for ( Term I : intervals ) {
			NumericalTerm est = intervalDomain.get(I).get(0).get(0).asNum();
			NumericalTerm eet = intervalDomain.get(I).get(1).get(0).asNum();
			
			if ( est.greaterThan(st) ) {
				st = est;
			}
			if ( eet.lessThan(et) ) {
				et = eet;
			}
			if ( et.lessThanEq(st) ) {
				return Term.bool(false);
			}
		}	

		return Term.bool(true);
	}

}
