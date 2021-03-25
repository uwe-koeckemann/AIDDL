package org.aiddl.common.math.graph;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class Graph {
	private Term G;
	private ListTerm inAdjList = null;	
	private ListTerm outAdjList = null;
	private ListTerm incidentList = null;
	
	public Graph ( Term G ) {
		this.G = G;
	}
	
	public Term getGraphTerm() {
		return G;
	}
	
	public CollectionTerm getNodes() {
		return this.G.get(GraphTerm.Nodes).asCollection();
	}
	
	public ListTerm getInEdges( Term node ) {
		if ( this.inAdjList == null ) {
			SetTerm V = G.get(GraphTerm.Nodes).asSet();
			SetTerm E = G.get(GraphTerm.Edges).asSet();
	
			LockableList adjList = new LockableList();
			
			Map<Term,LockableList> adjMap = new HashMap<>();
			
			for ( Term e : E ) {
				Term u = e.get(1);
				adjMap.putIfAbsent(u, new LockableList());
				adjMap.get(u).add(e);
			}
			
			for ( Term u : V ) {
				adjList.add(Term.keyVal(u, Term.list( adjMap.getOrDefault(u, new LockableList()))));
			}
			this.inAdjList = Term.list(adjList);
		}
		return this.inAdjList.getOrDefault(node, Term.list()).asList();
	}
	
	public ListTerm getOutEdges( Term node ) {
		if ( this.outAdjList == null ) {
			SetTerm V = G.get(GraphTerm.Nodes).asSet();
			SetTerm E = G.get(GraphTerm.Edges).asSet();
	
			LockableList adjList = new LockableList();
			
			Map<Term,LockableList> adjMap = new HashMap<>();
			
			for ( Term e : E ) {
				Term u = e.get(0);
				adjMap.putIfAbsent(u, new LockableList());
				adjMap.get(u).add(e);
			}
			
			for ( Term u : V ) {
				adjList.add(Term.keyVal(u, Term.list( adjMap.getOrDefault(u, new LockableList()))));
			}
			this.outAdjList = Term.list(adjList);
		}
		return this.outAdjList.get(node).asList();
	}
	
	public ListTerm getIncidentEdges( Term node ) {
		if ( this.incidentList == null ) {
			SetTerm V = G.get(GraphTerm.Nodes).asSet();
			SetTerm E = G.get(GraphTerm.Edges).asSet();
	
			LockableList adjList = new LockableList();
			
			Map<Term,LockableList> adjMap = new HashMap<>();
			
			for ( Term e : E ) {
				Term u = e.get(0);
				Term v = e.get(1);
				adjMap.putIfAbsent(u, new LockableList());
				adjMap.putIfAbsent(v, new LockableList());
				adjMap.get(u).add(e);
				adjMap.get(v).add(e);
			}
			
			for ( Term u : V ) {
				adjList.add(Term.keyVal(u, Term.list( adjMap.getOrDefault(u, new LockableList()))));
			}
			this.incidentList = Term.list(adjList);
		}
		return this.incidentList.get(node).asList();
	}
	
	public static NumericalTerm getWeight( Term weightTerm, Term v1, Term v2, FunctionRegistry fReg ) {
		if ( weightTerm instanceof FunctionReferenceTerm ) {
			Function f = weightTerm.asFunRef().getFunction();
			return f.apply(Term.tuple(v1, v2)).asNum();
		} 
		CollectionTerm w = weightTerm.asCollection();

		return w.getOrDefault(Term.tuple(v1, v2), w.getOrDefault(Term.set(v1, v2), Term.infPos())).asNum();
	}
}
