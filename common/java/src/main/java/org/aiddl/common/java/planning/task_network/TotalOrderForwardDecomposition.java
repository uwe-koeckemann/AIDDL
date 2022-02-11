package org.aiddl.common.java.planning.task_network;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.planning.state_variable.ApplicableFunction;
import org.aiddl.common.java.planning.state_variable.OperatorStateEnumerator;
import org.aiddl.common.java.planning.state_variable.StateTransitionFunction;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;

public class TotalOrderForwardDecomposition implements Function, InitializableFunction, ConfigurableFunction, InterfaceImplementation {
	
	public final static SymbolicTerm InterfaceURI = Term.sym("org.aiddl.common.planning.task-network.total-order-stn-planner");
	
	public Set<Term> primitiveTasks = new HashSet<Term>();
	SetTerm operators;
	SetTerm methods;
	
	Function applicable = new ApplicableFunction();
	Function stateTransition = new StateTransitionFunction();
	Function appEnumerator = new OperatorStateEnumerator();
	
	
	private int verbose = 0;
	private String loggerName = "TFD";
	
	/**
	 * Set verbose level of this function
	 * @param verbose verbosity level
	 */
	public void setVerbose( int verbose ) {
		this.verbose = verbose;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public void initialize(Term args) {
		operators = args.get(PlanningTerm.Operators).asSet();
		methods = args.get(PlanningTerm.Methods).asSet();
		for ( Term o : operators ) {
			this.primitiveTasks.add(o.get(PlanningTerm.Name).get(0));
		}
	}

	@Override
	public Term apply(Term args) {
		SetTerm s = args.get(PlanningTerm.InitialState).asSet();
		ListTerm openTasks = args.get(PlanningTerm.OpenTasks).asList();
		
		if ( openTasks.isEmpty() ) {
			return Term.list();
		}
		
		Term t1 = openTasks.get(0);
		if ( isPrimitive(t1) ) {
			if ( verbose >= 1 ) {
				Logger.msg(loggerName, "State: " + s);
				Logger.msg(loggerName, "Primitive task: " + t1);
				Logger.incDepth();
			}
			for ( Term a : operators ) {
				Substitution sig = t1.match(a.get(PlanningTerm.Name));
				
				if ( sig != null && applicable.apply(Term.tuple(a, s)).getBooleanValue() ) {
					Term s_next = stateTransition.apply(Term.tuple(s, a));
					

					
					LockableList w = new LockableList();
					for ( int i = 1 ; i < openTasks.size() ; i++ ) {
						w.add(openTasks.get(i).substitute(sig));
					}
					
					Term newOpenTasks = Term.list(w);
					
					if ( verbose >= 1 ) {
						Logger.msg(loggerName, "Trying: " + a.get(PlanningTerm.Name));
						Logger.msg(loggerName, "Open-tasks: " + newOpenTasks);						
					}
					
					Term pi = this.apply(
							Term.tuple(
									Term.keyVal(PlanningTerm.InitialState, s_next),
									Term.keyVal(PlanningTerm.OpenTasks, newOpenTasks)));
					
					if ( !pi.equals(CommonTerm.NIL) ) {
						LockableList pi_sol = new LockableList();
						pi_sol.add(a.get(PlanningTerm.Name));
						for ( Term a_pi : pi.asList() ) {
							pi_sol.add(a_pi);
						}
						return Term.list(pi_sol);
					}
				}
			}
			if ( verbose >= 1 ) {
				Logger.msg(loggerName, "Out of options.");
				Logger.decDepth();
			}
			return CommonTerm.NIL;
		} else {
			if ( verbose >= 1 ) {
				Logger.msg(loggerName, "State: " + s);
				Logger.msg(loggerName, "Non-primitive task: " + t1);
				Logger.incDepth();
			}
			for ( Term m : methods ) {		
				Substitution sub = m.get(PlanningTerm.Task).match(t1);				
				if ( sub != null ) {
					
					m = m.substitute(sub);

					SetTerm M_ground = this.appEnumerator.apply(Term.tuple(s, Term.set(m.substitute(sub)))).asSet();
					if ( verbose >= 1 ) {
						Logger.msg(loggerName, "Method: " + m.get(PlanningTerm.Name) + " has " + M_ground.size() + " applicable ground instance(s).");
					}
					
					for ( Term m_g : M_ground ) {
						Substitution task_sub = t1.match(m_g.get(PlanningTerm.Task));
						LockableList w = new LockableList();
						for ( Term subtask : m_g.get(PlanningTerm.SubTasks).asList() ) {
							w.add(subtask);
						}		
						for	( int i = 1 ; i < openTasks.size() ; i++ ) {
							w.add(openTasks.get(i).substitute(task_sub));
						}
						Term newOpenTasks = Term.list(w);
						if ( verbose >= 1 ) {
							Logger.msg(loggerName, "Trying: " + m_g.get(PlanningTerm.Name));
							Logger.msg(loggerName, "Open-tasks: " + newOpenTasks);						
						}
						Term pi = this.apply(
								Term.tuple(
										Term.keyVal(PlanningTerm.InitialState, s),
										Term.keyVal(PlanningTerm.OpenTasks, Term.list(w))));
						if ( !pi.equals(CommonTerm.NIL) ) {
							return pi;
						}
					}
				}
			}
			if ( verbose >= 1 ) {
				Logger.msg(loggerName, "Out of options.");
				Logger.decDepth();
			}
		}
		
		return CommonTerm.NIL;
	}

	private boolean isPrimitive( Term task ) {
		return  this.primitiveTasks.contains( task.get(0) ); 
	}

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceURI;
	}


	
}
