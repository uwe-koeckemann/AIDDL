package org.aiddl.example.learning_agent.functions;

import java.util.Map;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.common.java.planning.state_variable.Operator;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class OperatorExecutor implements ConfigurableFunction {
	
	private String name = this.getClass().getSimpleName();
	private boolean verbose = false;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		Term s = args.get(PlanningTerm.State);
		Term a = args.get(Term.sym("action"));
		CollectionTerm O = (CollectionTerm) args.get(PlanningTerm.Operators);
		
		Term s_next = null;
		for ( Term o : O ) {
			Term name = o.get(PlanningTerm.Name);
			Substitution sub = name.match(a);
			if ( sub != null ) {
				TupleTerm o_sub = (TupleTerm) o.substitute(sub);
				Operator svo = new Operator(o_sub);
				s_next = svo.applyTo((SetTerm) s);
				if ( verbose ) {
					Logger.msg(this.name, "Applying: " + name);
					Logger.msg(this.name, "  s  = " + s);
					Logger.msg(this.name, "  s' = " + s_next);
				}
				break;
			}
		}
		
		if ( s_next == null ) {
			if ( verbose ) Logger.msg(name, "Failure: Action " + a + " not applicable to " + s );
			s_next = s;
		} else {		
			if ( verbose ) Logger.msg(name, "Success: " + s + " x " + a + " -> " + s_next);
		}
	
		return s_next;
	}
}
