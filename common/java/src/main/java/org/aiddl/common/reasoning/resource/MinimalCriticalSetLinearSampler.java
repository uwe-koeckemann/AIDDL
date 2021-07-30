package org.aiddl.common.reasoning.resource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.aiddl.common.reasoning.temporal.TemporalTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.tools.TermComparator;

/**
 * Computes minimal critical sets for single resource by using linear sampling. This function does not 
 * apply a variable-ordering heuristic.
 * <p>
 * Input:
 * <ul>
 *   <li> RCPSP
 *   <li> Target resource
 *   <li> Propagated temporal interval domains
 * </ul>
 * <p>
 * Output:
 * <ul>
 *   <li> Collection of minimal critical sets
 * </ul>
 * <p>
 * Steps:
 * <ol> 
 *   <li> sort resource usages for target resource
 *   <li> sample peaks
 *   <li> return all identified peaks
 * </ol>
 * <p>
 * References:
 * <ul>
 *   <li> Cesta, A., Oddi, A., and Smith, S. F. (2002). A constraint-based method for project scheduling with time windows. Journal of Heuristics, 8(1):109-136.
 * </ul>
 *
 * @author Uwe Koeckemann
 *
 */
public class MinimalCriticalSetLinearSampler implements Function {

	@Override
	public Term apply(Term args) {
//		Term resource = args.get(0);
		SetTerm capacities = args.get(0).asSet();
		SetTerm allUsages = args.get(1).asSet();
		CollectionTerm intervalDomains = args.get(2).asCollection();
		
		//Logger.msg("MCS Sampler", Logger.prettyPrint(intervalDomains, 0));
		
		LockableSet peaks = new LockableSet();				
		
//		CollectionTerm usages = rcpsp.getOrPanic(ResourceTerm.Usage).getOrPanic(resource).asCollection();
//		NumericalTerm capacity = rcpsp.getOrPanic(ResourceTerm.Capacity).getOrPanic(resource).getOrPanic(Term.sym("max")).asNum();
	
		for ( Term cap : capacities ) {
			Term resource = cap.getKey();
			NumericalTerm capacity = cap.getValue().get(Term.sym("max")).asNum();
			SetTerm usages = allUsages.get(resource).asSet();
		
//			Function intersectCheck = new IntersectionChecker();
			
			List<Term> sortedUsages = new ArrayList<>();
			for ( Term u : usages ) {
				sortedUsages.add(u);
			}
			
			Collections.sort(sortedUsages,  new IntComp());	
			
			int i = 0;
			int j = 0;
			NumericalTerm current_consumption = Term.integer(0);
			NumericalTerm ST = null;
			NumericalTerm ET = null;
			
			LockableSet peak = new LockableSet();
			
			while ( i < sortedUsages.size() ) {
				if ( peak.isEmpty() ) {
					Term a_i = sortedUsages.get(i).getKey();
					peak.add(a_i);
					j = i+1;
					current_consumption = sortedUsages.get(i).getValue().asNum();
					ST = TemporalTerm.EST(a_i, intervalDomains);
					ET = TemporalTerm.EET(a_i, intervalDomains);
//					ST = intervalDomains.get(a_i).get(0).get(0).asNum();
//					ET = intervalDomains.get(a_i).get(1).get(0).asNum();
				} else {
					if ( j < sortedUsages.size() ) {
						Term a_j = sortedUsages.get(j).getKey();
						NumericalTerm u_j = sortedUsages.get(j).getValue().asNum();
										
						NumericalTerm a_j_est = TemporalTerm.EST(a_j, intervalDomains);
						NumericalTerm a_j_eet = TemporalTerm.EET(a_j, intervalDomains);

						NumericalTerm new_ST = ST.max(a_j_est);
						NumericalTerm new_ET = ET.min(a_j_eet);
						
						if ( new_ST.lessThan(new_ET) ) {
							peak.add(a_j);
							ST = new_ST;
							ET = new_ET;
							current_consumption = current_consumption.add(u_j);
							if ( current_consumption.greaterThan(capacity) ) {
								peaks.add(Term.set(peak));
								i++;
								peak = new LockableSet();
							}
						}
						j++;
					} else {
						i++;
						peak.clear();
					}
				}
			}
		}
		
		return Term.set(peaks);
	}
	
	private class IntComp implements Comparator<Term> {
		private TermComparator tComp = new TermComparator();
		
		@Override
		public int compare(Term o1, Term o2) {
			return -1*tComp.compare(o1.getValue(), o2.getValue());
		}
	}
}
