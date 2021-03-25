package org.aiddl.example.planning_with_resources.functions;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

public class ExpandResourceState implements Function {

	@Override
	public Term apply(Term args) {
		SetTerm r_assignments = args.get(0).getOrDefault(Term.sym("resource"), Term.set()).asSet();
		SetTerm state = args.get(1).asSet();
		
		LockableSet state_new = new LockableSet();
		
		for ( Term sva : state ) {
			if ( !r_assignments.containsKey(sva.getKey()) ) {
				state_new.add(sva);
			}
		}
		
		for ( Term r_a : r_assignments ) {
			Term resource = r_a.getKey();
			Term assignment = r_a.getValue();
			Term assignmentOperator = assignment.get(0);			
			NumericalTerm assignmentValue = assignment.get(1).asNum();
			NumericalTerm current = state.get(resource).asNum();
			if ( assignmentOperator.equals(Term.sym("+")) ) {
				current = current.add(assignmentValue);
			} else if ( assignmentOperator.equals(Term.sym("-")) ) {
				current = current.sub(assignmentValue);
			} else if ( assignmentOperator.equals(Term.sym("=")) ) {
				current = assignmentValue;
			}
			
//			Term new_value = state.get(resource).asTuple().put(PlanningTerm.Assignment, current);
			state_new.add(Term.keyVal(resource, current));
		}
		return Term.set(state_new);
	}
}
