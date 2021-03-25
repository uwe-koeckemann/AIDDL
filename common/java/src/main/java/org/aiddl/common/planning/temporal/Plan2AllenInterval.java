package org.aiddl.common.planning.temporal;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.Operator;
import org.aiddl.common.reasoning.temporal.TemporalTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;

/**
 * Convert a plan with durative actions to a set of quantified Allen Interval constraints.
 * 
 * @author Uwe Köckemann
 *
 */
public class Plan2AllenInterval implements Function, ConfigurableFunction, InterfaceImplementation {
	private String name = this.getClass().getSimpleName();
	private boolean verbose = false;
	
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.temporal.plan2allen-interval");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}


	@Override
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		SetTerm state = args.get(PlanningTerm.State).asSet();
		ListTerm plan = (ListTerm) args.get(PlanningTerm.Plan);
		SetTerm operators = (SetTerm) args.get(PlanningTerm.Operators);
		
		Term DefaultBound = Term.tuple(Term.integer(1), Term.infPos());
//		Term Duration = Term.sym("duration");
		
		Map<Term,Term> last_change = new HashMap<>();
		
		long nextFreeID = 0;
		
		for ( Term p : state ) {
			Term I = p.getValue().getOrDefault(Term.sym("I"), Term.tuple(Term.sym("I" + nextFreeID), p.getKey()));
			if ( p.get(Term.sym("int")) == null ) {
				nextFreeID++;		
			}
			last_change.put(p.getKey(), I);
		}
		
		
		LockableSet AIC = new LockableSet();
		
		if ( verbose ) Logger.incDepth();
		int current_action = 0;
		for ( Term action : plan ) {
			current_action++;
//			if ( current_action == 5 ) {
//				break;
//			}

			if ( verbose ) {
				Logger.decDepth();
				Logger.msg(this.name, current_action + ": " + action);
				Logger.incDepth();
			}
			
			
			for ( Term o_term : operators ) {
				Operator o = new Operator(o_term.asTuple());
				Substitution s = o.getName().match(action);
//				System.out.println(o.get(PlanningTerm.Name) + " <-> " + action + ": " + s);
				if ( s != null ) {
					
					
					Term id = o_term.get(Term.sym("id"));
//					Term interval = o_term.get(Term.sym("int"));
					if ( id != null ) {
						s.add(id, Term.integer(current_action));
					}					
					Term a_full = o_term.substitute(s);
//					System.out.println(a_full);
					
					Operator a = new Operator(a_full.asTuple());
					
					Term I_action = a_full.getOrDefault(Term.sym("I"), Term.tuple(Term.sym("A" + current_action), action.get(0)));
					
					
					Term duration = a_full.get(TemporalTerm.Duration);
					if ( duration != null ) {
					
						
						Term dur_con = Term.tuple(TemporalTerm.Duration, I_action, duration);
						AIC.add(dur_con);
						if ( verbose ) {
							Logger.msg(this.name, "Duration: " + dur_con);
						}
					}
					
					CollectionTerm Preconditions = a_full.get(PlanningTerm.Preconditions).asCollection();
					for ( Term p : Preconditions ) {
						Term I_pre = p.getValue().getOrDefault(Term.sym("I"), Term.tuple(Term.sym("P" + nextFreeID), p.getKey()));
						Term I_last = last_change.get(p.getKey());
						Term dur_con = Term.tuple(TemporalTerm.Equals, I_last, I_pre);
						AIC.add(dur_con);
					}	
					CollectionTerm Effects = a_full.get(PlanningTerm.Effects).asCollection();
					
					for ( Term e : Effects ) {
						Term I_eff = e.getValue().getOrDefault(Term.sym("I"), Term.tuple(Term.sym("E" + nextFreeID), e.getKey()));
						last_change.put(e.getKey(), I_eff);
					}
					
					Term Constraints = a_full.get(Term.sym("constraints"));
					if ( Constraints != null ) 
						Constraints = Constraints.get(Term.sym("temporal")); //org.aiddl.common.reasoning.temporal.allen-interval"));
					
					if ( Constraints != null ) {
						if ( verbose ) {
							Logger.msg(this.name, "Found explicit constraints:");
							Logger.incDepth();
							for ( Term c : Constraints.asCollection() ) {
								Logger.msg(this.name, c.toString());
							}
							Logger.decDepth();
						}
						for ( Term c : Constraints.asCollection() ) {
							AIC.add(c);
						}
					} else {
					
						for ( Term t : a_full.get(PlanningTerm.Preconditions).asCollection() ) {
	//						Term pre_lit = packLiteral(t);
							Term pre_var = t.getKey();
							
							Term I_pre = t.getValue().getOrDefault(Term.sym("I"), Term.tuple(Term.sym("P" + nextFreeID), t.getKey()));
					
							if ( id == null )
								nextFreeID++;
							Term I_causal_link = last_change.get(pre_var);
							
							Term causal_link = Term.tuple( Term.sym("equals"), I_pre, I_causal_link);
							Term action_link;
							
							if ( Effects.containsKey(pre_var) ) {
								action_link = Term.tuple( Term.sym("overlaps"), I_pre, I_action, DefaultBound);
							} else {
								action_link = Term.tuple( Term.sym("during"), I_action, I_pre, DefaultBound, DefaultBound);
							}
							AIC.add( causal_link );
							AIC.add( action_link );
							
							if ( verbose ) {
								Logger.msg(this.name, "Precondition: " + t + " Interval: " + I_pre);
								Logger.msg(this.name, "- Constraint: " + causal_link);
								Logger.msg(this.name, "- Constraint: " + action_link);
							}
						}
						
						for ( Term t : a.getEffects() ) {
							Term I_eff = t.getValue().getOrDefault(Term.sym("I"), Term.tuple(Term.sym("E" + nextFreeID), t.getKey()));
							if ( id == null )
								nextFreeID++;
							
							last_change.put(t.getKey(), I_eff);
								
							Term action_link = Term.tuple( Term.sym("meets"), I_action, I_eff);
							AIC.add(action_link);
							
							if ( verbose ) {
								Logger.msg(this.name, "Effect: " + t + " Interval: " + I_eff);
								Logger.msg(this.name, "- Constraint: " + action_link);
							}
						}
					}
					break;
				}
				
			}
		}
		if ( verbose ) Logger.decDepth();
		
		return Term.set(AIC);
	}
}
