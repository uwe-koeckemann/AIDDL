package org.aiddl.common.data;

import org.aiddl.common.CommonTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class Set implements Function {

	java.util.Set<Term> S = new java.util.HashSet<>();
	
	static Term Add = Term.sym("add");
	static Term AddAll = Term.sym("add-all");
	
	static Term Contains = Term.sym("contains");
	static Term ContainsAll = Term.sym("contains-all");
	
	@Override
	public Term apply(Term args) {

		Term operator = args.get(0);
		Term arg = args.get(1);
		
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(Add) ) {
			S.add(arg);
		} else if ( operator.equals(AddAll) ) {
			for ( Term e : arg.asCollection() ) {
				S.add(e);
			}
		} else if ( operator.equals(Contains) ) {
			r = Term.bool(S.contains(arg));
		} else if ( operator.equals(ContainsAll) ) {
			for ( Term e : arg.asCollection() ) {
				if ( !S.contains(e) ) {
					r = Term.bool(false);
					break;
				}
			}
			r = Term.bool(true);
		} 
		
		return r;
	}
}
