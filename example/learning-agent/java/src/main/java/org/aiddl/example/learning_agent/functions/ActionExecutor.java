package org.aiddl.example.learning_agent.functions;

import java.util.Map;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class ActionExecutor implements ConfigurableFunction {
	
	private String name = this.getClass().getSimpleName();
	private boolean verbose = false;
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		Term a = args.get(Term.sym("action"));
		Term s = args.get(PlanningTerm.State);
		Term pair = Term.tuple(a,s);
		
		Map<Term,Term> Sigma = ((CollectionTerm)args.get(Term.sym("sigma"))).getMap();
		
		
		Term s_next = Sigma.get(pair);
	
		if ( s_next == null ) {
			if ( verbose ) Logger.msg(name, "Failure: Action " + a + " not applicable to " + s );
			s_next = s;
		} else {		
			if ( verbose ) Logger.msg(name, "Success: " + s + " x " + a + " -> " + s_next);
		}
	System.out.println(s_next);
		return s_next;
	}
}
