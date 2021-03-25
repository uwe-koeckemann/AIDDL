package org.aiddl.common.reasoning.resource;

import java.util.Collections;

import org.aiddl.common.reasoning.temporal.TemporalTerm;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.TermComparator;

public class LfVariableMfValueOrdering implements PureFunction, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.resource.variable-value-ordering");
	

	private static final Term DefaultBounds = Term.tuple(Term.integer(0), Term.infPos());
	
	@Override
	public Term apply(Term args) {
		CollectionTerm peaks = args.get(0).asCollection();
		Term intervalDomains = args.get(1);
		
		PureFunction flexCalc = new FlexibilityLossFunction();
		
		Term argMax = null;
		NumericalTerm max = Term.infNeg();
		
		for ( Term peak : peaks ) {
			Term k_pc = flexCalc.apply(Term.tuple(peak, intervalDomains));
			
			NumericalTerm k = k_pc.get(0).asNum();
			if ( k.greaterThan(max) ) {
				max = k;
				argMax = k_pc.get(1);
			}
		}
		LockableList sorted = new LockableList();
		for ( Term e : argMax.asCollection() ) {
			sorted.add(e);
		}
		Collections.sort(sorted, new TermComparator());
		
		LockableList resolvers = new LockableList();
		for ( Term e : sorted ) {
			resolvers.add(Term.tuple(TemporalTerm.Before, e.get(1), e.get(2), DefaultBounds));
		}
		
		return Term.list(resolvers);
	}

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
}
