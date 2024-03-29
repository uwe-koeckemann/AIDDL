package org.aiddl.common.java.planning.state_variable;

import java.util.Map;

import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableSet;

/**
 * Combines: 
 * 1) grounding based on state (ApplicableOperatorEnumerator), 
 * 2) grounding open variables based on domains (Operator Enumerator)
 * 3) evaluating result and checking if constraints satisfied
 * 
 * @author Uwe Koeckemann
 *
 */
public class OperatorStateConstrainedDomainEnumerator implements Function, ConfigurableFunction, InitializableFunction {
	
	OperatorStateEnumerator apOpEnum;
	OperatorDomainEnumerator opEnum;
	Evaluator eval;
	SetTerm domains;
	
	public OperatorStateConstrainedDomainEnumerator() {};
	
	public OperatorStateConstrainedDomainEnumerator( SetTerm domains, Evaluator eval ) {
		apOpEnum = new OperatorStateEnumerator();
		opEnum = new OperatorDomainEnumerator();
		this.domains = domains;
		this.eval = eval;		
	}
	

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.eval = (Evaluator) fReg.getFunction(Uri.EVAL);
	}
	
	@Override
	public void initialize(Term args) {
		apOpEnum = new OperatorStateEnumerator();
		opEnum = new OperatorDomainEnumerator();
		this.domains = args.asSet();
	}

	@Override
	public Term apply(Term args) {
		Term state = args.get(0);
		SetTerm actions = args.get(1).asSet();
		
		SetTerm applicable_actions = apOpEnum.apply(Term.tuple(state, actions)).asSet();
		
		LockableSet feasible = new LockableSet();
		
		for ( Term a : applicable_actions ) {
			for ( Term a_ground : opEnum.apply(Term.tuple(a, domains)).asCollection() ) {
				a_ground = eval.apply(a_ground);
				if ( !a_ground.getOrDefault(Term.sym("constraints"), Term.set()).asCollection().contains(Term.bool(false)) ) {
					feasible.add(a_ground);
				}
			}
		}
		return Term.set(feasible);
	}








	
	
}
