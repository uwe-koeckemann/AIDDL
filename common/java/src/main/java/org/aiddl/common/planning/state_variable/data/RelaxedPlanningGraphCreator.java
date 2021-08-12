package org.aiddl.common.planning.state_variable.data;

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.state_variable.Operator;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;

public class RelaxedPlanningGraphCreator implements InterfaceImplementation, ConfigurableFunction {
	
	private static final String name = "RelaxedPlanningGraphCreator";
	private int verbose = 0; 
		
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.data.rpg-creator");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}

	@Override
	public Term apply( Term problem ) {
		SetTerm O = (SetTerm) problem.get(PlanningTerm.Operators);
		SetTerm s = (SetTerm) problem.get(PlanningTerm.InitialState);
		SetTerm g = (SetTerm) problem.get(PlanningTerm.Goal);
		
		Set<Operator> A = new LinkedHashSet<>();
		for ( Term o : O ) {
			A.add(new Operator((TupleTerm) o));
		}
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		
		Term O_ground = groundOps.compute(A, s);
		A.clear();
		for ( Term o : O_ground.asCollection() ) {
			A.add(new Operator(o.asTuple())); 
		}	
		return this.compute(A, s, g);
	}
	
	public ListTerm compute ( Set<Operator> A, SetTerm s, SetTerm g ) {
		LockableList layers = new LockableList();
				
		Set<Operator> unusedActions = new HashSet<>();
		unusedActions.addAll(A);
		Set<Term> newPropositions = new HashSet<>();
		Set<Operator> newActions = new HashSet<>(); 
		newPropositions.addAll( s.getLockedSet() );
				
		layers.add(s);
		int currentLayer = 0;
		boolean done = false;
		while ( !done ) {
			currentLayer = layers.size()-1;
			if ( verbose >= 1 ) Logger.msg(name, "Working on layer: " + currentLayer + " with " + newPropositions.size() + " new propositions and " + newActions.size() + " new actions.");
			
			LockableSet actionLayer = new LockableSet();
			LockableSet propositionLayer = new LockableSet();
			newActions = new HashSet<>();
			
			// New NOOPs
			for ( Term p : newPropositions ) {	
				actionLayer.add( Operator.assembleOperator(Term.tuple(Term.sym("NOOP"), p.getKey(), p.getValue()), Term.set(p), Term.set(p)));
			}
			newPropositions.clear();
			newActions.clear();
			// Previous actions
			if ( currentLayer > 1 ) {
				actionLayer.addAll(layers.get(currentLayer-1).asSet().getLockedSet());
			}
			// Previous propositions
			propositionLayer.addAll(layers.get(currentLayer).asSet().getLockedSet());
			
			if ( verbose >= 2 ) Logger.incDepth();
			for ( Operator a : unusedActions ) {
				if ( layers.get(currentLayer).asCollection().containsAll(a.getPreconditions()) ) {
					if ( verbose >= 2 ) {
						Logger.msg(name, "New Action: " + a);
						Logger.incDepth();
					}
					
					newActions.add(a);
					for ( Term e : a.getEffects() ) {
						if (  !layers.get(currentLayer).asCollection().contains(e) ) {
							newPropositions.add(e);
							if ( verbose >= 2 ) Logger.msg(name, "New proposition: " + e);
						}
					}
					propositionLayer.addAll(a.getEffects().getLockedSet());
					actionLayer.add(a.getOperatorTerm());
					
					if ( verbose >= 2 ) Logger.decDepth();
				}
			}
			if ( verbose >= 2 ) Logger.decDepth();
			
			layers.add( Term.set(actionLayer) );
			layers.add( Term.set(propositionLayer) );
		
			unusedActions.removeAll(newActions);
			
			if ( newPropositions.isEmpty() && newActions.isEmpty() ) {
				if ( verbose >= 1 ) Logger.msg(name, "Found fixed point.");
				done =  true;
			}
			if ( propositionLayer.containsAll(g.getLockedSet()) ) {
				if ( verbose >= 1 ) Logger.msg(name, "Found all goals.");
				done = true;
			}
		}
		return Term.list(layers);
	}	
}