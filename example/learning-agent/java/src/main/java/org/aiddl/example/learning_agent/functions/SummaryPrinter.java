package org.aiddl.example.learning_agent.functions;

import java.util.Map;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class SummaryPrinter implements ConfigurableFunction {
	
	String name = SummaryPrinter.class.getSimpleName();
	boolean verbose = false;
	
	@Override
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}
	
	@Override
	public Term apply( Term args ) {
		SetTerm O = (SetTerm) args.get(PlanningTerm.Operators);
		SetTerm Data = (SetTerm) args.get(LearningTerm.Data);
		
		Term planTerm = args.get(PlanningTerm.Plan);
		ListTerm Plan;
		if ( planTerm.equals(Term.sym("NIL")) ) {
			Plan = Term.list(); 
		} else {
			Plan = (ListTerm) args.get(PlanningTerm.Plan);
		}
		
		if ( verbose ) {
			Logger.msg(name, "|Data| = " + Data.size() + " |O| = " + O.size() + " |pi| = " + Plan.size());
		}
 
		return null;
	}
}
