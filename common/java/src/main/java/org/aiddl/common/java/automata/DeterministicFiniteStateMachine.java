package org.aiddl.common.java.automata;

import java.util.Map;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;

/**
 * Maintain and advance Deterministic Finite State Automata.
 *
 * @author Uwe Koeckemann
 */
public class DeterministicFiniteStateMachine implements Function, InitializableFunction, ConfigurableFunction {
	int verbose = 0;
	Term ignoredEvent = Term.sym("none");
	Term defaultBehavior = Term.sym("panic");
	
	Term dfa;
	
	SetTerm states;
	SetTerm finalStates;
	SetTerm transitions;
	SetTerm events;
	Term initialState;
	
	Term currentState;
		
	@Override
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
//		this.ignoredEvent = settings.getOrDefault(Term.sym("ignoredEvent"), ignoredEvent);
		this.defaultBehavior = settings.getOrDefault(Term.sym("defaultBehavior"), defaultBehavior);
	}
	
//	private static final Term StateKey = Term.sym("state");
//	private static final Term IsFinalStateKey = Term.sym("accepted");
	

	@Override
	public void initialize(Term arg) {
		dfa = arg;
		states = dfa.get(0).asSet();
		events = dfa.get(1).asSet();
		transitions = dfa.get(2).asSet();
		initialState = dfa.get(3);
		finalStates = dfa.get(4).asSet();
		currentState = initialState;
	}
	
	private static final Term Panic = Term.sym("panic");
	private static final Term Ignore = Term.sym("ignore");

	
	private static final Term Step = Term.sym("step");
	private static final Term MultiStep = Term.sym("multi-step");

	private static final Term CurrentState = Term.sym("current-state");
	private static final Term IsFinalState = Term.sym("is-final-state");
	
	private static final Term Reset = Term.sym("reset");
	
	@Override
	public Term apply( Term args ) {
		Term r = CommonTerm.NIL;
			
		if ( args instanceof TupleTerm ) {
			if ( args.get(0).equals(Step) ) {
				Term event = args.get(1);
				r = this.step(event);
			} else if ( args.get(0).equals(MultiStep) ) {
				for ( Term event : args.get(1).asList() ) {
					r = this.step(event);
				}
			}
		} else if ( args.equals(IsFinalState) ) {
			r = Term.bool(finalStates.contains(currentState));		
		} else if ( args.equals(CurrentState) ) {
			r = currentState;		
		} else if ( args.equals(Reset) ) {
			currentState = initialState;			
		} 

		return r;
	}
	
	private Term step( Term e ) {
		if ( !events.contains(e) ) {
			if ( defaultBehavior.equals(Panic)) {
				System.err.println("DFA:\n" + Logger.prettyPrint(dfa, 1));
				throw new IllegalArgumentException("Bad input: " + e + " for DFA and default behavior is set to panic.");
			}
		} else {
			Term nextState = this.transitions.get(Term.tuple(currentState, e));
			if ( nextState == null ) {
				if ( defaultBehavior.equals(Panic)) {
					System.err.println("DFA:\n" + Logger.prettyPrint(dfa, 1));
					throw new IllegalArgumentException("Undefined state transition: " + Term.tuple(currentState, e) + " for DFA and default behavior is set to panic.");
					
				}
			} else {
				currentState = nextState;
				return nextState;
			}
		}
		return currentState;
	}
}
