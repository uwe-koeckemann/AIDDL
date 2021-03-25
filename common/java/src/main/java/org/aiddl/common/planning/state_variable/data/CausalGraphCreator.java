package org.aiddl.common.planning.state_variable.data;

import java.util.HashSet;
import java.util.Set;

import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.common.math.graph.GraphTools;
import org.aiddl.common.math.graph.StronglyConnectedComponents;
import org.aiddl.common.planning.state_variable.Operator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;

public class CausalGraphCreator implements Function, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.data.causal-graph-creator");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply(Term O) {
		LockableSet V = new LockableSet();
		LockableList E = new LockableList();
		
		for ( Term o_term : O.asCollection() ) {
			Operator o = new Operator(o_term.asTuple());
//			CollectionTerm effects = o.get(PlanningTerm.Effects).asCollection();
//			CollectionTerm preconditions = o.get(PlanningTerm.Preconditions).asCollection();
			
			for ( Term e1 : o.getEffects() ) {
//				if ( e1 instanceof TupleTerm ) {
//					e1 = e1.getOrDefault(PlanningTerm.StateVariableAssignment, e1);
//				}
				V.add(e1.getKey());
				for ( Term e2 : o.getEffects() ) {
					if ( !e1.getKey().equals(e2.getKey()) ) {
						E.add(Term.tuple(e1.getKey(), e2.getKey()));
					}
				}
				for ( Term p : o.getPreconditions() ) {
					V.add(p.getKey());
					E.add(Term.tuple(p.getKey(), e1.getKey()));
				}
			}
			
		}
		
		Graph G = new Graph( GraphTools.assembleGraph(Term.set(V), Term.set(E)) );
//		DirectedGraph2Dot g2d = new DirectedGraph2Dot();
//		g2d.setExportFilename("causal-graph.dot");
//		g2d.compute(G.getGraphTerm());
		
		StronglyConnectedComponents sccComp = new StronglyConnectedComponents();
		SetTerm SCCs = sccComp.compute(G).asSet();
		
		LockableSet cycle_free_edges = new LockableSet(); 
	    cycle_free_edges.addAll( G.getGraphTerm().get(GraphTerm.Edges).getSetCopy() );
		
		for ( Term SCC : SCCs ) {
			
			ListTerm order = totalOrder(SCC.asSet(), G);
			
			for ( Term v1 : SCC.asSet() ) {
				for ( Term e : G.getOutEdges(v1) ) {
					Term v2 = e.get(1);
					if ( SCC.asSet().contains(v2) ) {
						if ( order.indexOf(v1) >= order.indexOf(v2) ) {
							cycle_free_edges.remove(e);
						}
					}
				}
			}
		}
		
		Term G_causal = GraphTools.assembleGraph(Term.set(V), Term.set(cycle_free_edges));
		
//		g2d.setExportFilename("causal-graph-cycle-free.dot");
//		g2d.compute(G_causal);
		
		return G_causal;
	}
	
	private ListTerm totalOrder( SetTerm SCC, Graph G ) {
		LockableList order = new LockableList();

		Set<Term> ignored_edges = new HashSet<>();
		while ( order.size() != SCC.size() ) {
			Integer min = null;
			Term argMin = null;
			
			for ( Term u : SCC ) {
				if ( !order.contains(u)  ) {
					int in_counter = 0;
					
					for ( Term e : G.getInEdges(u) ) {
						if ( !ignored_edges.contains(e) ) {
							Term source = e.get(0);
					
							if ( !SCC.contains(source) ) {
								ignored_edges.add(e);
							} else {
								in_counter++;
							}
						}
					}
					if ( min == null || in_counter < min ) {
						min = in_counter;
						argMin = u;
					}
				}
			}
			order.add(argMin);
			ignored_edges.addAll(G.getIncidentEdges(argMin).getLockedList());
		}
		return Term.list(order);
	}
}
