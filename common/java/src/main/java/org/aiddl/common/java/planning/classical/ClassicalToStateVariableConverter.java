package org.aiddl.common.java.planning.classical;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.java.planning.state_variable.ForwardSearchPlanner;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;
import org.aiddl.core.java.tools.Logger;

/**
 * Convert Classical planning problem to state variable planning problem.
 * @author Uwe Koeckemann
 *
 */
public class ClassicalToStateVariableConverter implements ConfigurableFunction {		
	String name = "C->SV Planning";
	int verbose = 0;

	Map<Term, Term> settings = new HashMap<>();

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.settings = settings;
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Term apply( Term problem ) {			
		SetTerm O = (SetTerm) problem.get(PlanningTerm.Operators);
		SetTerm s0 = (SetTerm) problem.get(PlanningTerm.InitialState);
		SetTerm g = (SetTerm) problem.get(PlanningTerm.Goal);
		
		LockableSet s0_svp = new LockableSet();
		for ( Term p : s0 ) {
			s0_svp.add(Term.keyVal(p, Term.bool(true)));
			
			if ( verbose >= 2 ) Logger.msg("ClassicalPlanningSolver", p + " -> " + Term.keyVal(p, Term.bool(true)));
		}
		
		LockableSet g_svp = new LockableSet();
		for ( Term goal : g ) {
			g_svp.add(this.literal2keyval(goal));
			
			if ( verbose >= 2 ) Logger.msg("ClassicalPlanningSolver", goal + " -> " + this.literal2keyval(goal));
		}
		
		LockableSet O_svp = new LockableSet();
		for ( Term o : O ) {
			LockableList o_svp = new LockableList();
			o_svp.add(Term.keyVal(PlanningTerm.Name, o.get(PlanningTerm.Name)));
			
			LockableSet P_svp = new LockableSet();
			LockableSet E_svp = new LockableSet();
			
			for ( Term p : o.get(PlanningTerm.Preconditions).asCollection() ) {
				P_svp.add(literal2keyval(p));
			}
			for ( Term e : o.get(PlanningTerm.Effects).asCollection() ) {
				E_svp.add(literal2keyval(e));
			}
			o_svp.add(Term.keyVal(PlanningTerm.Preconditions, Term.set(P_svp)));
			o_svp.add(Term.keyVal(PlanningTerm.Effects, Term.set(E_svp)));
		
			O_svp.add(Term.tuple(o_svp));
			
			if ( verbose >= 2 ) Logger.msg("ClassicalPlanningSolver", o + "\n -> " + Term.tuple(o_svp));
		}

		ForwardSearchPlanner fsp = new ForwardSearchPlanner();
		fsp.configure(settings, null);
		
		Term svpProblem = Term.tuple(
				Term.keyVal(PlanningTerm.Operators, Term.set(O_svp)),
				Term.keyVal(PlanningTerm.InitialState, Term.set(s0_svp)),
				Term.keyVal(PlanningTerm.Goal, Term.set(g_svp)) );
		
		return svpProblem;
	}
	
	private Term literal2keyval( Term l ) {
		if ( l instanceof TupleTerm && l.size() > 1 ) {
			if ( l.get(0).equals(Term.sym("+")) ) {
				return Term.keyVal(l.get(1), Term.bool(true));
			} else if ( l.get(0).equals(Term.sym("-")) ) {
				return Term.keyVal(l.get(1), Term.bool(false));
			} else {
				return Term.keyVal(l, Term.bool(true));
			}
		}
		return Term.keyVal(l, Term.bool(true));
	}
}
