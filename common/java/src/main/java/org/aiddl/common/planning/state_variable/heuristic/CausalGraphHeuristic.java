package org.aiddl.common.planning.state_variable.heuristic;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.common.planning.PlanningHeuristic;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.tools.Profiler;
import org.aiddl.core.tools.StopWatch;

/**
 * Implementation of the Causal Graph heuristic.
 * 
 * @see Helmert, M., The fast downward planning system, Journal of Artificial Intelligence Research, 26(1), 191–246 (2006). 
 * 
 * @author Uwe Koeckemann
 *
 */
public class CausalGraphHeuristic implements InitializableFunction, ConfigurableFunction, InterfaceImplementation, PlanningHeuristic {
	
	public static final Term Unknown = Term.sym("#unknown#");
	
	String name = CausalGraphHeuristic.class.getSimpleName();
	int verbose = 0;
	
	private Graph CG;
	private Map<Term, Graph> DTGs;
	private SetTerm g;

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.heuristic");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public void initialize(Term args) {
		Term CG_term = args.get(PlanningTerm.CausalGraph);
		Term DTGs_term = args.get(PlanningTerm.DomainTransitionGraphs);
				
		this.g = args.get(PlanningTerm.Goal).asSet();
		this.CG = new Graph( CG_term );
		this.DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}		
	}
	
	public void setDataStructures( Graph CG, Map<Term, Graph> DTGs ) {
		this.CG = CG;
		this.DTGs = DTGs;
	}
	public void setGoal( SetTerm g ) {
		this.g = g;
	}
	
	@Override
	public Term apply(Term args) {
		return compute(args.asSet(), this.g);
	}
		
	@Override
	public NumericalTerm compute( SetTerm s, SetTerm g ) {
//		Map<Term, Term> svaMap = PlanningTerm.convert2svaMap(s);
		NumericalTerm h = Term.integer(0);
		if ( verbose >= 2 ) {
			Logger.msg(name, "Cost from " + s + " to " + g);
			Logger.incDepth();
		}
	
		for ( Term g_entry : g ) {
			Term variable = g_entry.getKey();
			Term current_value = s.getOrDefault(variable, Unknown);
				
			Term goal_value = g_entry.getValue();
			
			NumericalTerm h_part = this.compute(s, variable, current_value, goal_value);
			
			if ( verbose >= 3 ) Logger.incDepth();
			
			h = h.add(h_part);
			
			if ( verbose >= 3 ) {
				Logger.decDepth();
				Logger.msg(name, "Changing " + variable + " from " + current_value + " to " + goal_value + " costs " + h_part);			
			}
			
			if ( h.equals(Term.infPos()) ) {
				break;
			}
		}
		if ( verbose >= 2 ) {
			Logger.decDepth();
			Logger.msg(name, "heuristic value: " + h);
		}
		return h;
	}
		
	private Map<Term, Map<Term,NumericalTerm>> cost_cache = new HashMap<>();
	private Map<Term, ListTerm> predecessorCache = new HashMap<>();

	public NumericalTerm compute( SetTerm s, Term x, Term v_current, Term v_target ) {
//		Profiler.probe(0);
//		if ( verbose >= 2 ) Logger.msg(name, x + " from " + v_current + " to " + v_target);
		/**
		 * 1) Local state for x (based on predecessors in causal graph)
		 * 2) Init. costs for values of x (0 for current, inf else)
		 * 3) Until done:
		 *   - Select node
		 *   - forall out edges in DTG
		 *     - sum recursive cost calculations from DTG conditions that overlap with local state
		 *     - update cost if better than record
		 * 4) return lowest cost to target 
		 */
		
		if ( v_current.equals(v_target) ) {
//			if ( verbose >= 2 ) Logger.msg(name, "heuristic value: 0");
			return Term.integer(0);
		}
		
		TupleTerm contextID = Term.tuple(s, x, v_current);
		
		NumericalTerm cost = null;
		cost_cache.putIfAbsent(contextID, new HashMap<>());
		Map<Term, NumericalTerm> cost_cache_in_context = cost_cache.get(contextID);
		if ( (cost = cost_cache_in_context.get(v_target)) != null ) {
//			if ( verbose >= 2 ) Logger.msg(name, "heuristic value: " + cost);
//			Profiler.probe(0);
			return cost;
		}
		
//		Map<Term, Term> sva_map = PlanningTerm.convert2svaMap(s);
		Map<Term, SetTerm> localStateMap = new HashMap<>();
		SetTerm localState = this.getLocalState(CG, s, x, v_current);
		localStateMap.put(v_current, localState);
		localStateMap.put(Unknown, localState);

		Graph DTG = DTGs.get(x);

		if ( DTG == null ) {
			return Term.infPos();
		}
		Set<Term> unreached = new HashSet<>();
//		LinkedList<Term> unreachedQueue = new LinkedList<>();
		
		for ( Term v : DTG.getNodes() ) {
			cost_cache_in_context.put(v, (v.equals(v_current) || v.equals(Unknown)) ? Term.integer(0) : Term.infPos() );
			unreached.add(v);
//			unreachedQueue.add(v);
		}
//		Collections.sort(unreachedQueue, comp);
		
//		if ( verbose >= 2 ) {
//			Logger.msg(name, "Unreached:  " + unreached);
//			Logger.msg(name, "First next: " + chooseNext(contextID, unreached));
//		}
		
		
		NumericalTerm costToTarget = Term.infPos();
		Term d_i = CommonTerm.NIL;
		while ( !unreached.isEmpty() && d_i != null ) {
			d_i = chooseNext(cost_cache_in_context, unreached);
			if ( d_i == null ) break;
			unreached.remove(d_i);
			SetTerm localState_d_i = localStateMap.get(d_i);
			
//			if ( verbose >= 2 ) {
//				Logger.msg(name, "d_i:  " + d_i);
//				Logger.msg(name, "localState_d_i:  " + localState_d_i);
//				Logger.msg(name, "Out Edges: " + DTG.getOutEdges(d_i));
//				Logger.msg(name, "DTG:" + DTG.getGraphTerm());
//			}
			SetTerm conditionLookup = DTG.getGraphTerm().get(GraphTerm.Labels).asSet();
			
			for ( Term e : DTG.getOutEdges(d_i) ) {
				Term d_j = e.get(1);
				
				SetTerm conditions = conditionLookup.get(e).asSet();
				
				NumericalTerm transition_cost = Term.integer(1);
				
//				if ( verbose >= 3 ) {
//					Logger.msg(name, "Conditions:  " + conditions);
//					Logger.msg(name, "Local state: " + localState_d_i);
//				}
					
				
				for ( Term entry : conditions ) {
					Term e_var = entry.getKey(); //get(PlanningTerm.StateVariable);
					Term e_current = localState_d_i.get(e_var);	// where we are in local state
					Term e_target  = entry.getValue();    					// where we need to be to satisfy condition
					
//					System.out.println("--------");
//					System.out.println(entry);
//					System.out.println(e_current);
//					System.out.println(e_target);
					
					if ( e_current != null ) {
						if ( verbose >= 3 ) Logger.incDepth();
						NumericalTerm sub_cost = this.compute(s, e_var, e_current, e_target);
						if ( verbose >= 3 ) Logger.decDepth();
						
						transition_cost = transition_cost.add(sub_cost);
						
						if ( transition_cost.equals(Term.infPos()) ) {
							break;
						}
					}
				}
			
				NumericalTerm cost_to_d_i = cost_cache_in_context.get(d_i);
				NumericalTerm cost_to_d_j = cost_to_d_i.add(transition_cost);
			
//				if ( verbose >= 3 ) {
//					Logger.msg(name, "" + transition_cost);
//					Logger.msg(name, "Cost to d_i: " + cost_to_d_i);
//					Logger.msg(name, cost_to_d_j + " < " + cost_to_);
//					Logger.msg(name, "Current cost: " + cost_cache.get(contextID).get(d_j));
//				}
				
				
				
				if ( cost_to_d_j.lessThan(cost_cache_in_context.get(d_j)) ) {
//					if ( verbose >= 3 ) {
//						Logger.msg(name, "Updating: "+x+" from " + d_i + " to " + d_j + " with cost " + cost_to_d_j);
//					}
						
					cost_cache_in_context.put(d_j, cost_to_d_j);
					
					if ( d_j.equals(v_target) ) {
						costToTarget = cost_to_d_j;
					}
					
//					System.out.println("A: " + localState_d_i);
//					System.out.println("B: " + conditions);
					LockableSet local_d_j = new LockableSet();
//					Map<Term, Term> svaMap_cond = PlanningTerm.convert2svaMap(conditions); 
//					SetTerm localState_d_j = localState_d_i;
					for ( Term sva : localState_d_i ) {
						if ( !conditions.containsKey(sva.getKey())) {
							local_d_j.add(sva);
						} 
					}
					for ( Term kvp : conditions.asSet() ) {
						Term var = kvp.getKey();
						Term assignment = kvp.getValue();
						local_d_j.add(Term.keyVal(var, assignment));
					}
					
//					SetTerm localState_d_j = localState_d_i.putAll(conditions);
					localStateMap.put(d_j, Term.set(local_d_j)); //localState_d_j);
					
				} 
//				else {
//					if ( verbose >= 3 ) {
//						Logger.msg(name, "Not Updating: "+x+" from " + d_i + " to " + d_j + " with min " + cost_cache.get(contextID).get(d_j));
//					}
//				}
			}		
		}
//		if ( verbose >= 2 ) Logger.msg(name, "Final heuristic value for "+x+" from " + v_current + " to " + v_target + " is " + costToTarget);
		return costToTarget;
	}
	
	private Term chooseNext( Map<Term, NumericalTerm> cost_cache, Set<Term> unreached ) {
		Term argMin = null;
		NumericalTerm min = Term.infPos();
			
		for ( Term v : unreached ) {
//			StopWatch.start("GET");
			NumericalTerm cost = cost_cache.get(v);
//			StopWatch.stop("GET");
//			if ( verbose >= 3 ) {
//				Logger.msg(this.name, "Term: " + v + " has cost " + cost);
//			}
//			StopWatch.start("MIN");
			if ( cost.lessThan(min) ) {
				min = cost;
				argMin = v;
			}
//			StopWatch.stop("MIN");
		}
		return argMin;
	}
	
	private SetTerm getLocalState( Graph CG, SetTerm s, Term variable, Term value  ) {
		LockableSet s_local = new LockableSet();
		for ( Term pre_var : this.getPredecessors(CG, variable)) {
			s_local.add(Term.keyVal(pre_var, s.getOrDefault(pre_var, Unknown)));
		}
		return Term.set(s_local);		
	}
	
	private ListTerm getPredecessors( Graph CG, Term variable ) {
		ListTerm predecessors = this.predecessorCache.get(variable);
		if ( predecessors == null ) {
			LockableList preLlist = this.getPredecessorsHelper(CG, variable);
			predecessors = Term.list(preLlist);
			this.predecessorCache.put(variable, predecessors);
		}
		return predecessors;
	}
	
	private LockableList getPredecessorsHelper( Graph CG, Term variable ) {
		LockableList pre = new LockableList();
		for ( Term e : CG.getInEdges(variable) ) {
			Term source = e.get(0);
			pre.add(source);
			// This made things slow without changing the outcome. Double check if this is needed
			//pre.addAll(this.getPredecessorsHelper(CG, source));
		}
		return pre;
	}
}
