package org.aiddl.common.reasoning.resource;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.aiddl.common.reasoning.temporal.TemporalTerm;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class FlexibilityLossFunction implements PureFunction {

	@Override
	public Term apply(Term args) {
		SetTerm peak = args.get(0).asSet();
		CollectionTerm intervalDomain = args.get(1).asCollection();
		
		
		List<Term[]> mcslist = new ArrayList<Term[]>();
		
		Set<Term> added = new HashSet<>();
		
		for ( Term a : peak ) {
			added.add(a);
			for ( Term b : peak ) {
				if ( !added.contains(b) ) {
					Term[] oneMcs = new Term[2];
					oneMcs[0] = a;
					oneMcs[1] = b;
					mcslist.add(oneMcs);
				}
			}
		}
		
		LockableList r = new LockableList();
		List<NumericalTerm> pc_vec = new ArrayList<>();
		NumericalTerm pc_min = Term.real(1.1);
		
		for ( Term[] pair : mcslist ) {
			boolean a_works = true;
			boolean b_works = true;
			
			
//			NumericalTerm pc_min_bad = Term.real(1.1); 
			
			
			NumericalTerm i_est = TemporalTerm.EST(pair[0], intervalDomain); //.get(0).get(0).asNum();
			NumericalTerm i_lst = TemporalTerm.LST(pair[0], intervalDomain); // .get(0).get(1).asNum();//.min(Term.integer(Long.MAX_VALUE));
			NumericalTerm i_eet = TemporalTerm.EET(pair[0], intervalDomain);
			NumericalTerm i_let = TemporalTerm.LET(pair[0], intervalDomain);//.min(Term.integer(Long.MAX_VALUE));;
			NumericalTerm j_est = TemporalTerm.EST(pair[1], intervalDomain); //.get(0).get(0).asNum();
			NumericalTerm j_lst = TemporalTerm.LST(pair[1], intervalDomain); //.get(0).get(1).asNum();//.min(Term.integer(Long.MAX_VALUE));;
			NumericalTerm j_eet = TemporalTerm.EET(pair[1], intervalDomain);
			NumericalTerm j_let = TemporalTerm.LET(pair[1], intervalDomain);//.min(Term.integer(Long.MAX_VALUE));;				
					
			NumericalTerm dmin = j_est.sub(i_let);
			NumericalTerm dmax = j_lst.sub(i_eet);
			NumericalTerm pc_a = Term.real(0.0);
			NumericalTerm pc_b = Term.real(0.0);
			
//			System.out.println("dmin: " + dmin + " dmax: " + dmax);
			
			if ( !dmin.equalTo(dmax) ) {
				if ( dmin.isNegative() && dmax.isNegative() ) {
					pc_a = Term.real(1.0);
				} else if ( dmin.equals(Term.infNeg()) && dmax.equals(Term.infPos()) ) {
					pc_a = Term.real(0.5);
				}  else {
					NumericalTerm a = dmax.min(Term.real(0.0)).sub(dmin.min(Term.real(0.0)));
					NumericalTerm b = dmax.sub(dmin); 
					pc_a = a.div(b);
				}
				pc_vec.add(pc_a);
				if ( pc_a.lessThan(pc_min) ) {
					pc_min = pc_a;
				}
			} else {
				a_works = false;
			}
			dmin = i_est.sub(j_let);
			dmax = i_lst.sub(j_eet);
			
//			System.out.println("dmin: " + dmin + " dmax: " + dmax);
				
			if ( !dmin.equalTo(dmax) ) {	
				if ( dmin.isNegative() && dmax.isNegative() ) {
					pc_b = Term.real(1.0);
				} else if ( dmin.equals(Term.infNeg()) && dmax.equals(Term.infPos()) ) {
					pc_b = Term.real(0.5);
				} else {
					NumericalTerm a = dmax.min(Term.real(0.0)).sub(dmin.min(Term.real(0.0)));
					NumericalTerm b = dmax.sub(dmin); 
					pc_b = a.div(b);
//					pc = (dmax.min(Term.integer(0)).sub(dmin.min(Term.integer(0)))).div(dmax.sub(dmin));
				}
				pc_vec.add(pc_b);	
				if (pc_b.lessThan(pc_min) ) {
					pc_min = pc_b;
				}
			} else {
				b_works = false;
			}				
		
			if ( a_works ) {
				r.add(Term.tuple(pc_a, pair[0], pair[1]));
			}
			if ( b_works ) {
				r.add(Term.tuple(pc_b, pair[1], pair[0]));
			}
		}
		
		NumericalTerm k = Term.real(0.0);

		for( int i = 0; i < pc_vec.size(); i++ ) {
			k = k.add(Term.real(1.0).div(Term.real(1.0).add(pc_vec.get(i).sub(pc_min)))); // 1.0f/(1.0f + pc_vec.get(i) - pc_min);
		}
		if ( ! k.isZero() ) {
			k = Term.real(1.0).div(k);
		} else {
			k = Term.real(1.0);
		}
				
		return Term.tuple(k, Term.list(r));
	}

}
