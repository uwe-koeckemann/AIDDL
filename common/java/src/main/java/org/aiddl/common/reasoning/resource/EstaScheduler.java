package org.aiddl.common.reasoning.resource;

import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.temporal.allen_constraints.Allen2STP;
import org.aiddl.common.reasoning.temporal.allen_constraints.IntersectionChecker;
import org.aiddl.common.reasoning.temporal.allen_constraints.Timepoint2IntervalDomain;
import org.aiddl.common.reasoning.temporal.simple_temporal.STPSolver;
import org.aiddl.common.search.TreeSearch;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.Logger;

public class EstaScheduler implements Function, ConfigurableFunction, InitializableFunction {

	Allen2STP allen2stp = new Allen2STP(); 
	STPSolver stpSolver = new STPSolver(); 
	IntersectionChecker intersectionCheck = new IntersectionChecker();
	Timepoint2IntervalDomain time2interval = new Timepoint2IntervalDomain();
	MinimalCriticalSetLinearSampler peakCollector = new MinimalCriticalSetLinearSampler();
	
	FlexibilityLossFunction valueOrdering = new FlexibilityLossFunction();
	LfVariableMfValueOrdering variableValueORdering = new LfVariableMfValueOrdering();
	
	TreeSearch search = new TreeSearch();
	
	Term a;
	Term a_solution = null;
	NumericalTerm a_count;
	
	SetTerm AC;
	
	SetTerm constraints;
	
	Term capacity;
	Term usage;
	
	Term STP;
	Term intervals;
	
	String name = "PlanIterator";
	int verbose = 0;
	
	public void setVerbose( int lvl ) {
		this.verbose = lvl;
	}
	
	@Override
	public void initialize(Term rcpsp) {
		constraints = rcpsp.get(ResourceTerm.Constraints).asSet();
		capacity = rcpsp.get(ResourceTerm.Capacity);
		usage = rcpsp.get(ResourceTerm.Usage);
		
		a = Term.list();
		a_solution = null;
		a_count = Term.integer(0);
	}

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
			this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Term apply(Term args) {
		Term peaks;
		
		if ( args.get(0).equals(Term.sym("next")) ) {
			if ( this.a_solution != null ) {
				this.a = search.apply(Term.tuple(Term.sym("next")));
				findNextConsistentAssignment();
			} else if ( this.a_solution == null ) {
				AC = constraints.addAll(a.asCollection());
				STP = allen2stp.apply(AC);
				intervals = stpSolver.apply(STP);
			} else if ( this.a_solution.equals(CommonTerm.NIL ) ) {
				return this.a_solution;
			}

			// Find and remove peaks
			do {
				peaks = peakCollector.apply(Term.tuple(capacity, usage, intervals)).asSet();
				if ( this.verbose >= 2 ) {
					Logger.msg(name, "Assignment: " + a);
					Logger.msg(name, "Peaks: " + peaks);
				}
					
				
				if ( peaks.size() > 0 ) {
					Term expansion = variableValueORdering.apply(Term.tuple(peaks, intervals));
					if ( this.verbose >= 2 ) Logger.msg(name, "Expansion: " + expansion);
					
					a = search.apply(Term.tuple(Term.sym("expand"), expansion));
					findNextConsistentAssignment();
				}
			} while ( !a.equals(CommonTerm.NIL) && peaks.size() > 0 );
			a_solution = a;
		}
		if ( this.verbose >= 1 ) {
			Logger.msg(name, "Solution: " + a_solution);
		}
		
		return a_solution;
	}
	
	// Search for next consistent assignment
	private void findNextConsistentAssignment() {
		do {
			this.a_count = a_count.add(Term.integer(1));
			this.AC = constraints.addAll(a.asCollection());
			this.STP = allen2stp.apply(AC);
			this.intervals = stpSolver.apply(STP);
			
			if ( intervals.equals(CommonTerm.NIL) ) {
				this.a = this.search.apply(Term.tuple(Term.sym("next")));
			}
		} while ( intervals.equals(CommonTerm.NIL) && !a.equals(CommonTerm.NIL) );
	}
	
}
