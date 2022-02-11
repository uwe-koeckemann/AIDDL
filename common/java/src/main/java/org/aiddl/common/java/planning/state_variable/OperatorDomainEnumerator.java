package org.aiddl.common.java.planning.state_variable;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.VariableTerm;
import org.aiddl.core.java.tools.ComboIterator;
import org.aiddl.core.java.tools.LockableSet;

public class OperatorDomainEnumerator implements Function {

	@Override
	public Term apply(Term args) {
		Term o = args.get(0);
		Term D = args.get(1);
		LockableSet O_ground = new LockableSet();
		
		Term name = o.getOrPanic(PlanningTerm.Name);
		Term sig = o.getOrPanic(PlanningTerm.Signature);
		
		List<List<Term>> choices = new ArrayList<>();
		for ( int i = 0 ; i < name.size() ; i++ ) {
			Term o_arg = name.get(i);
			if ( o_arg instanceof VariableTerm ) {

				Term type_name = sig.getOrPanic(o_arg);
				Term domain = D.getOrPanic(type_name);
				List<Term> d_choice = new ArrayList<>(domain.size());
				for ( Term value : domain.asCollection() ) {
					d_choice.add(Term.keyVal(o_arg, value));
				}
				choices.add(d_choice);
			}			
		}
		
		if ( !choices.isEmpty() ) {
			ComboIterator<Term> comboIterator = new ComboIterator<>(choices);
			for ( List<Term> sub_term : comboIterator ) {
				Substitution sub = new Substitution();
				for ( Term st : sub_term ) {
					sub.add(st.getKey(), st.getValue());
				}
				O_ground.add(o.substitute(sub));
			}
		} else {
			O_ground.add(o);
		}
		
		
		
		
		return Term.set(O_ground);
	}
}
