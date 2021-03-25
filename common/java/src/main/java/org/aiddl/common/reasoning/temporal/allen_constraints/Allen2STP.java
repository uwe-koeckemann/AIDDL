package org.aiddl.common.reasoning.temporal.allen_constraints;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.aiddl.common.reasoning.temporal.TemporalTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

public class Allen2STP implements Function, InterfaceImplementation {
	
	private Term O = Term.sym("O");
//	private Term H = Term.sym("H");
//	private long INF = Long.MAX_VALUE/2-2;
		
	private HashSet<Term> unaryConstraints = new HashSet<>();
		

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.temporal.allen-interval.allen-2-stp");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	public Allen2STP() {
		unaryConstraints.add(TemporalTerm.Release);
		unaryConstraints.add(TemporalTerm.Deadline);
		unaryConstraints.add(TemporalTerm.Duration);
		unaryConstraints.add(TemporalTerm.At);
	}
	
	@Override
	public Term apply(Term AIC) {
		List<Term> X = new ArrayList<Term>();
		X.add(O);
//		X.add(H);
		
		List<TupleTerm> C = new ArrayList<>();
		
		for ( Term c : AIC.asSet() ) {
			Term constraintName = c.get(0);
			if ( unaryConstraints.contains(constraintName) ) {
				unaryAllen2SimpleDistance( (TupleTerm) c, X, C );
			} else {
				binaryAllen2SimpleDistance( (TupleTerm) c, X, C );
			}
		}
		
		return Term.tuple(Term.set(X), Term.set(C));
	}
	
	private NumericalTerm min( NumericalTerm a, NumericalTerm b ) {
		if ( a.lessThanEq(b) ) {
			return a;
		}
		return b;
	}
	
	private void unaryAllen2SimpleDistance( TupleTerm c, List<Term> X, List<TupleTerm> C ) {
		Term constraintName = c.get(0);
		Term interval = c.get(1);
		Term tp1_s = Term.tuple(Term.sym("ST"), interval);
		Term tp1_e = Term.tuple(Term.sym("ET"), interval);
		
		if ( !X.contains(tp1_s) ) {
			X.add(tp1_s);
			X.add(tp1_e);
			C.add( Term.tuple(tp1_s, tp1_e, Term.integer(0), Term.infPos()) );
		}
		
		Term b1_l = min(Term.infPos(), c.get(2).get(0).asNum());
		Term b1_u = min(Term.infPos(), c.get(2).get(1).asNum());
		Term b2_l = null;
		Term b2_u = null;
		
		if ( c.size() > 3 ) {
			b2_l = min(Term.infPos(), c.get(3).get(0).asNum());
			b2_u = min(Term.infPos(), c.get(3).get(1).asNum());
		}
		if ( constraintName.equals(TemporalTerm.Release) ) {
			C.add( Term.tuple(O, tp1_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.Deadline) ) {
			C.add( Term.tuple(O, tp1_e, b1_l, b1_u) );
		}  else if ( constraintName.equals(TemporalTerm.Duration) ) {
			C.add( Term.tuple(tp1_s, tp1_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.At) ) {
			C.add( Term.tuple(O, tp1_s, b1_l, b1_u) );
			C.add( Term.tuple(O, tp1_e,   b2_l, b2_u) );
		} 
	}

	private void binaryAllen2SimpleDistance( TupleTerm c, List<Term> X, List<TupleTerm> C ) {
		Term constraintName = c.get(0);
		Term interval_1 = c.get(1);
		Term interval_2 = c.get(2);
		
		Term tp1_s = Term.tuple(Term.sym("ST"), interval_1);
		Term tp1_e = Term.tuple(Term.sym("ET"), interval_1);
		Term tp2_s = Term.tuple(Term.sym("ST"), interval_2);
		Term tp2_e = Term.tuple(Term.sym("ET"), interval_2);
		
		if ( !X.contains(tp1_s) ) {
			X.add(tp1_s);
			X.add(tp1_e);
			C.add( Term.tuple(tp1_s, tp1_e, Term.integer(0), Term.infPos()) );
		}
		if ( !X.contains(tp2_s) ) {
			X.add(tp2_s);
			X.add(tp2_e);
			C.add( Term.tuple(tp1_s, tp2_e, Term.integer(0), Term.infPos()) );
		}
	
		Term b1_l = null;
		Term b1_u = null;
		
		if ( c.size() > 3 ) {
			b1_l = min(Term.infPos(), c.get(3).get(0).asNum());
			b1_u = min(Term.infPos(), c.get(3).get(1).asNum());
		}
		
		Term b2_l = null;
		Term b2_u = null;
		
		if ( c.size() > 4 ) {
			b2_l = min(Term.infPos(), c.get(4).get(0).asNum());
			b2_u = min(Term.infPos(), c.get(4).get(1).asNum());
		}
		
		if ( constraintName.equals(TemporalTerm.Before) ) {	
			C.add( Term.tuple(tp1_e, tp2_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.After) ) {		
			C.add( Term.tuple(tp2_e, tp1_s, b1_l, b1_u) );
		} else if (constraintName.equals(TemporalTerm.Equals) ) {
			C.add( Term.tuple(tp1_s, tp2_s, Term.integer(0), Term.integer(0)) );
			C.add( Term.tuple(tp1_e, tp2_e, Term.integer(0), Term.integer(0)) );
		} else if ( constraintName.equals(TemporalTerm.Meets) ) {
			C.add( Term.tuple(tp1_e, tp2_s, Term.integer(0), Term.integer(0)) );
		} else if ( constraintName.equals(TemporalTerm.MetBy) ) {
			C.add( Term.tuple(tp1_s, tp2_e, Term.integer(0), Term.integer(0)) );
		} else if ( constraintName.equals(TemporalTerm.Starts) ) {
			C.add( Term.tuple(tp1_s, tp2_s, Term.integer(0), Term.integer(0)) );
			C.add( Term.tuple(tp1_e, tp2_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.StartedBy) ) {
			C.add( Term.tuple(tp1_s, tp2_s, Term.integer(0), Term.integer(0)) );
			C.add( Term.tuple(tp2_e, tp1_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.During) ) {
			C.add( Term.tuple(tp2_s, tp1_s, b1_l, b1_u) );
			C.add( Term.tuple(tp1_e, tp2_e, b2_l, b2_u) );
		} else if ( constraintName.equals(TemporalTerm.Contains) ) {			
			C.add( Term.tuple(tp1_s, tp2_s, b1_l, b1_u) );
			C.add( Term.tuple(tp2_e, tp1_e, b2_l, b2_u) );
		} else if ( constraintName.equals(TemporalTerm.Finishes) ) {	
			C.add( Term.tuple(tp1_e, tp2_e, Term.integer(0), Term.integer(0)) );
			C.add( Term.tuple(tp2_s, tp1_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.FinishedBy) ) {			
			C.add( Term.tuple(tp1_e, tp2_e, Term.integer(0), Term.integer(0)) );
			C.add( Term.tuple(tp1_s, tp2_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.Overlaps) ) {			
			C.add( Term.tuple(tp1_s, tp2_s, Term.integer(1), Term.infPos()) );
			C.add( Term.tuple(tp1_e, tp2_e, Term.integer(1), Term.infPos()) );
			C.add( Term.tuple(tp2_s, tp1_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.OverlappedBy) ) {	
			C.add( Term.tuple(tp2_s, tp1_s, Term.integer(1), Term.infPos()) );
			C.add( Term.tuple(tp2_e, tp1_e, Term.integer(1), Term.infPos()) );
			C.add( Term.tuple(tp1_s, tp2_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.StSt) ) {	
			C.add( Term.tuple(tp1_s, tp2_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.StEt) ) {	
			C.add( Term.tuple(tp1_s, tp2_e, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.EtSt) ) {	
			C.add( Term.tuple(tp1_e, tp2_s, b1_l, b1_u) );
		} else if ( constraintName.equals(TemporalTerm.EtEt) ) {	
			C.add( Term.tuple(tp1_e, tp2_e, b1_l, b1_u) );
		}
//			else if ( constraintName.equals(TemporalTerm.OverlapsAtLeast) ) {
//			C.add( Term.tuple(tp2_s, tp1_e, b1_l, b1_u) );
//		} 
	}	
}
