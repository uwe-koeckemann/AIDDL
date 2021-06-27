package org.aiddl.common.planning.state_variable;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;

public class OperatorReachableEnumerator implements ConfigurableFunction {
		
	String name = OperatorReachableEnumerator.class.getSimpleName();
	int verbose = 0;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Term apply(Term Pi) {
		SetTerm O = (SetTerm) Pi.get(PlanningTerm.Operators);
		SetTerm s0 = (SetTerm) Pi.get(PlanningTerm.InitialState);
//		SetTerm g = (SetTerm) Pi.get(Term.sym(":goals"));
		Set<Operator> A = new LinkedHashSet<>();
		for ( Term o : O ) {
			A.add(new Operator((TupleTerm) o));
		}
		return this.compute(A, s0);
	}
	

	public Term compute( Set<Operator> O, SetTerm s0 ) {
		Set<Operator> O_ground = groundOperators(O, s0);
		
		LockableSet O_r = new LockableSet();	
		for ( Operator o : O_ground ) {	// TODO: avoid this by using LockableSet to collect before
			O_r.add(o.getOperatorTerm());
		}
		if ( verbose >= 1 ) {
			Logger.msg(this.name, "Number of ground operators: " + O_r.size());
			if ( verbose >= 2 ) {
				Logger.incDepth();
				for ( Term o : O_r ) {
					Logger.msg(this.name, o.get(PlanningTerm.Name).toString());
				}
				Logger.decDepth();
			}
		}
		return Term.set(O_r);
	}
	
	private Set<Operator> groundOperators( Set<Operator> O, SetTerm s0 ) {
		Map<Term,Set<Term>> s_acc = new HashMap<>();
		for ( Term e : s0 ) {
			Term e_sv = e.getKey();
			s_acc.put(e_sv , new LinkedHashSet<>());
			s_acc.get(e_sv).add(e.getValue());
		}

		Set<Operator> O_acc = new LinkedHashSet<>();

		int O_size_prev = -1;
		while ( O_size_prev != O_acc.size() ) {
			O_size_prev = O_acc.size();
			Map<Term, Set<Term>> s_new = new HashMap<>();
			for ( Operator o : O ) {
				Set<Operator> O_ground = getGround(o, s_acc);
				
				for ( Operator o_ground : O_ground ) {
					SetTerm eff = o_ground.getEffects();
					for ( Term e : eff ) {
						Term e_sv = e.getKey();
						s_new.putIfAbsent(e_sv, s_acc.getOrDefault(e_sv, new LinkedHashSet<>()));
						s_new.get(e_sv).add(e.getValue());
					}				
				}
				O_acc.addAll(O_ground);
				
			}
			s_acc.putAll(s_new);
		}

		
		return O_acc;
	}
	
	private Set<Operator> getGround( Operator o, Map<Term, Set<Term>> s_acc ) {
		Set<Operator> O_ret = new HashSet<>();
		if ( o.getName().isGround() ) {
			boolean pre_sat = true;
			for ( Term e : o.getPreconditions() ) {
				if ( !s_acc.getOrDefault(e.getKey(), new LinkedHashSet<>()).contains(e.getValue()) ) {
					pre_sat = false;
					break;
				}
			}
			if ( pre_sat ) {
				boolean valid_operator = true;
				Set<Term> test = new HashSet<Term>();
				for ( Term e : o.getEffects() ) {
					Term e_sv = e.getKey();
					if ( test.contains(e_sv) ) {
						valid_operator = false;
						break;
					}
					test.add(e_sv);
				}
				if ( valid_operator ) {
					test.clear();
					for ( Term p : o.getPreconditions() ) {
						Term p_sv = p.getKey();
						if ( test.contains(p_sv) ) {
							valid_operator = false;
							break;
						}
						test.add(p_sv);
					}
				}
				if ( valid_operator ) {
					O_ret.add(o);
				} 
				return O_ret;
			} else {
				return new HashSet<Operator>();
			}
		} else {
			for ( Term pre_entry : o.getPreconditions() ) {
				Term pre_sv = pre_entry.getKey();
				Term pre_a  = pre_entry.getValue();				
				if ( !pre_sv.isGround() || !pre_a.isGround() ) {
					for ( Term s_key : s_acc.keySet() ) {
						Substitution sub_key = pre_sv.match(s_key);
						if ( sub_key != null ) {
							for ( Term s_val : s_acc.get(s_key)) {
								Substitution sub_val = pre_a.match(s_val);
								if ( sub_val != null ) {
									Substitution sub = sub_key.copy();
									if ( sub.add(sub_val) ) {
										Operator o_subbed = o.substitute(sub);
										O_ret.addAll(getGround(o_subbed, s_acc));
										
									}
								}
							}
						}
					}
					break;
				}
			}
			return O_ret;
		}
	}
}
