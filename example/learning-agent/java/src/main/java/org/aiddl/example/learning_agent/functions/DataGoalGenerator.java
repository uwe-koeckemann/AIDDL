package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;

public class DataGoalGenerator implements ConfigurableFunction {

	Random rng;
	int verbose = 0;
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		rng = new Random();
	}
	
	Term var = Term.anonymousVar();
	private final static Term COLLECTED = Term.sym("collected"); 
	private final static Term DATA = Term.sym("data");
		
	@Override
	public Term apply( Term args ) {
		CollectionTerm locations = (CollectionTerm) args.get(Term.sym("locations"));
		CollectionTerm configs = (CollectionTerm) args.get(Term.sym("configs"));
		CollectionTerm data = (CollectionTerm) args.get(LearningTerm.Data);
			
		Term matchTerm; 
		
		List<Term> goal_candidates = new ArrayList<>();

		for ( Term l : locations ) {
			for ( Term c : configs ) {
				matchTerm = Term.tuple(l,c, Term.anonymousVar());
				boolean hasMatch = false;
				for ( Term t : data ) {
					if ( matchTerm.match(t) != null ) {
						hasMatch = true;
						break;
					}
				}
				if ( !hasMatch ) {
					goal_candidates.add(
							Term.keyVal(Term.tuple(COLLECTED, Term.tuple(DATA, l, c)),
										Term.sym("true")));
				}
			}
		}

		LockableSet goal_selected = new LockableSet();
		
		while ( goal_selected.size() < 1 && !goal_selected.containsAll(goal_candidates) ) {
			goal_selected.add(goal_candidates.get(rng.nextInt(goal_candidates.size())));
		}
		
		if ( verbose > 1 ) {
			Logger.msg("GoalSelector", "Selected " +goal_selected.size()+ " goals.");
			
		}
		
		return Term.set( goal_selected );
	}
}
