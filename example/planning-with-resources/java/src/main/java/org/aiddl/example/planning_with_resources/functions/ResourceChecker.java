package org.aiddl.example.planning_with_resources.functions;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class ResourceChecker implements ConfigurableFunction {
	
	private String name = this.getClass().getSimpleName();
	private boolean verbose = false;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		CollectionTerm r0 = args.get(0).asCollection();
		ListTerm plan = args.get(1).asList();
		Term constraints = args.get(2);
		SetTerm O = args.get(3).asSet();		
		
		Map<Term, LinkedList<NumericalTerm>> resource_profile = new HashMap<>();
		Map<Term, NumericalTerm> min_value = new HashMap<>();
		Map<Term, NumericalTerm> max_value = new HashMap<>();
		
		for ( Term r0_entry : r0.asCollection() ) {
			if ( verbose ) {
				Logger.msg(name, "Setting initial resource state: " + r0_entry);
			}
			Term resource = r0_entry.getKey();
			LinkedList<NumericalTerm> profile = new LinkedList<>();
			profile.add((NumericalTerm)r0_entry.getValue());
			resource_profile.put(resource, profile);
			
			NumericalTerm min = constraints.get(resource).get(Term.sym("min")).asNum();
			NumericalTerm max = constraints.get(resource).get(Term.sym("max")).asNum();
			
			min_value.put(resource, min);
			max_value.put(resource, max);
		}
		
		int step = 0;
		boolean consistent = true;
		for ( Term action : plan ) {
			step += 1;
			for ( Term o : O ) {
				Term o_name = o.get(PlanningTerm.Name);
				Substitution s = o_name.match(action);
				if ( s != null ) {
					Term o_sub = o.substitute(s);
					for ( Term r_usage : o_sub.getOrDefault(Term.sym("resource"), Term.set()).asCollection() ) {
						Term resource = r_usage.getKey();
						Term change = r_usage.getValue();
						if ( !resource_profile.containsKey(resource) ) {
							throw new IllegalArgumentException("Resource " + resource + " used in action:\n" + o_sub + "\nnot part of initial resource setting." );
						}
						NumericalTerm current = resource_profile.get(resource).getLast();
						
						if ( change.get(0).equals(Term.sym("+")) ) {
							current = current.add((NumericalTerm)change.get(1));
						} else if ( change.get(0).equals(Term.sym("-")) ) {
							current = current.sub((NumericalTerm)change.get(1));
						} else if ( change.get(0).equals(Term.sym("=")) ) {
							current = (NumericalTerm)change.get(1);
						} else {
							throw new IllegalArgumentException("Unsupported resource change " + change + ".\n"
									+ "\tUse (= n), (+ n), or (- n) to set, increase, or decrese current value to/by numerical n.");
						}
						
						if ( current.greaterThan(max_value.get(resource)) 
						  || current.lessThan(min_value.get(resource)) ) {
							consistent = false;
						}
						
						resource_profile.get(resource).addLast(current);
					}
					// Advance all resources not used by operator.
					for ( Term resource : resource_profile.keySet() ) {
						if ( resource_profile.get(resource).size() < step+1 ) {
							resource_profile.get(resource).add(resource_profile.get(resource).getLast());
						}
						
					}
					break;
				}
			}
		}
		
		if ( verbose ) {
			Logger.msg(name, "Profile consistent? " + consistent);
			Logger.incDepth();
			for ( Term r0_kvp : r0 ) {
				Logger.msg(name, r0_kvp.getKey() + " " + constraints.get(r0_kvp.getKey()) + resource_profile.get(r0_kvp.getKey()));
			}
			Logger.decDepth();
		}
	
		return Term.bool(consistent);
	}
}
