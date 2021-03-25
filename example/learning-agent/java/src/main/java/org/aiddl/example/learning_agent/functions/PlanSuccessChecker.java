package org.aiddl.example.learning_agent.functions;

import java.util.Random;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.Logger;

public class PlanSuccessChecker implements Function {

	Random r;
	int next_action = 0;
		
	@Override
	public Term apply( Term args ) {
		SetTerm goal = (SetTerm) args.get(PlanningTerm.Goal);
		SetTerm state = (SetTerm) args.get(PlanningTerm.State);
		
		boolean success = state.containsAll(goal);
		
		if ( success ) {
			Logger.msg(this.getClass().getSimpleName(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>");
			Logger.msg(this.getClass().getSimpleName(), ">>> GOAL REACHED BY PLAN!");
			Logger.msg(this.getClass().getSimpleName(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>");
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
		} else {
			Logger.msg(this.getClass().getSimpleName(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>");
			Logger.msg(this.getClass().getSimpleName(), ">>> GOAL FAILED...");
			Logger.msg(this.getClass().getSimpleName(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>");
		}
		return Term.bool(success);
	}	
}
