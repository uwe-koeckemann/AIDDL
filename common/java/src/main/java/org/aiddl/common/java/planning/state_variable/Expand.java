package org.aiddl.common.java.planning.state_variable;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class Expand implements Function, InitializableFunction, ConfigurableFunction, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.expand");
	
	SetTerm operators;
	
	List<Function> added_transitions = new LinkedList<>();
	
	static final Term EXPAND_HOOKS = Term.sym("expand-hooks");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		if ( settings.containsKey(EXPAND_HOOKS) ) {
			for ( Term fun_name : settings.get(EXPAND_HOOKS).asCollection() ) {
				added_transitions.add(fReg.getFunction(fun_name));
			}
		}
	}
	
	@Override
	public void initialize(Term args) {
		operators = args.asSet();
	}
	
	@Override
	public Term apply(Term args) {
		SetTerm state = args.asSet();
//		HashMap<Term, Term> svaMap = new HashMap<>();
//		HashMap<Term, Term> svaMapFull = new HashMap<>();
		
//		for ( Term s_e : state ) {
//			svaMap.put(s_e.get(PlanningTerm.StateVariable), s_e);
//			svaMapFull.put(s_e.get(PlanningTerm.StateVariable), s_e);
//		}
		
		LockableList expansion = new LockableList();
		
		for ( Term o : operators ) {
			if ( applicable(o, state) ) {
				Term succ_state = apply(o, state);
				for ( Function f_trans : added_transitions ) {
					succ_state = f_trans.apply(Term.tuple(o, succ_state));
				}
				expansion.add(Term.tuple(o.get(PlanningTerm.Name), succ_state));
			}
		}
		return Term.list(expansion);
	}
	
	private boolean applicable( Term o, SetTerm s ) {	
		for ( Term p : o.get(PlanningTerm.Preconditions).asCollection() ) {
			Term p_sv = p.getKey();
			Term p_a = p.getValue();
			Term s_a = s.get(p_sv);
			if ( s_a == null || !s_a.equals(p_a) ) {
				return false;
			}
		}
		return true;
	}
	
	private SetTerm apply( Term o, SetTerm s ) {	
		return s.putAll(o.get(PlanningTerm.Effects).asSet());
	}
}
