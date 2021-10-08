package org.aiddl.common.planning.state_variable;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.ComboIterator;
import org.aiddl.core.tools.LockableSet;

/**
 * Find all ways to apply non-ground operators to a state
 * @author Uwe Koeckemann
 *
 */
public class OperatorStateEnumerator implements Function {

	@Override
	public Term apply(Term args) {
		SetTerm s = args.get(0).asSet();
		SetTerm O = args.get(1).asSet();
		
		LockableSet O_applicable = new LockableSet();

		for ( Term o : O ) {
			List<List<Substitution>> choices = new ArrayList<>();
			for ( Term p : o.get(PlanningTerm.Preconditions).asSet() ) {
				Term p_sv = p.getKey();
				Term p_a = p.getValue();
				List<Substitution> p_choice = new ArrayList<>();
				
				for ( Term s_e : s ) {
					Term s_sv = s_e.getKey();
					Term s_a = s_e.getValue();
						Substitution sub_sv = p_sv.match(s_sv);
						if ( sub_sv != null ) {
							Substitution sub_a = p_a.match(s_a);
							if ( sub_sv.add(sub_a) ) {
								p_choice.add(sub_sv);
							}
						}
				}
				choices.add(p_choice);
			}
			ComboIterator<Substitution> comboIt = new ComboIterator<>(choices);
			for ( List<Substitution> combo : comboIt ) {
				Substitution build = new Substitution();
				for ( Substitution sub : combo ) {
					if ( !build.add(sub) ) {
						build = null;
						break;
					}
				}
				if ( build != null ) {
					O_applicable.add(o.substitute(build));
				}
			}
		}
		
		
		return Term.set(O_applicable);
	}

}
