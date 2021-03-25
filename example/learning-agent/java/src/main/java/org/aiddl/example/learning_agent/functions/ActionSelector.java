package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;

public class ActionSelector implements ConfigurableFunction {

	Random r;
	
	private boolean verbose = false;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		r = new Random();
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}
	
	public final static Term ActionKey = Term.sym("selected-action");
	public final static Term RemainingPlanKey = Term.sym("plan-tail");
		
	@Override
	public Term apply( Term args ) {
		Term plan = args.get(PlanningTerm.Plan);
		Term selected_action = null;
		if ( !plan.equals(Term.sym("NIL")) ) { 
			ListTerm planList = (ListTerm) plan;
			if ( !(planList.size() == 0) ) {
				selected_action = planList.get(0);
				plan = Term.list(planList.getListCopy().subList(1, planList.size()));
				if ( verbose ) Logger.msg(this.getClass().getSimpleName(), "Planned action: " + selected_action + " remaining plan: " + plan);
			}
		} 
		if ( selected_action == null ) {
			Term s = args.get(PlanningTerm.State);
			
			Map<Term,Term> Sigma = ((ListTerm)args.get(Term.sym("sigma"))).getMap();
			
			CollectionTerm actions = (CollectionTerm) args.get(Term.sym("actions"));
		
			Term pair = null;
			List<Term> applicable_actions = new ArrayList<>();
			System.out.println(Sigma);
			for ( Term a : actions ) {
//				pair = Term.tuple(a,s);
//				System.out.println("Asking for: " + pair);
//				if ( Sigma.get(pair) != null ) {
//					System.out.println(Sigma.get(pair));
//					applicable_actions.add(a);
//				}
				applicable_actions.add(a);
			}
			selected_action = applicable_actions.get(r.nextInt(applicable_actions.size()));
			pair = Term.tuple(selected_action,s);
			
			if ( verbose ) Logger.msg(this.getClass().getSimpleName(), "Experimental action: " + selected_action);
		}
		
		LockableList rList = new LockableList();
		rList.add(Term.keyVal(ActionKey, selected_action));
		rList.add(Term.keyVal(RemainingPlanKey, plan));
		return Term.list(rList);
	}
}
