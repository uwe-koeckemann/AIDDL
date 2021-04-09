package org.aiddl.common.math.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.LockableList;

public class StronglyConnectedComponents implements Function, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.math.graph.compute-scc");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply( Term G ) {
		return compute( new Graph(G) );
	}
		
	public Term compute( Graph G ) {
		DepthFirstSearch DFS = new DepthFirstSearch();
		Transpose trans = new Transpose();
					
		Term dfs_result = DFS.compute(G);
		SetTerm finish = dfs_result.get(DepthFirstSearch.FinishKey).asSet();
		
		List<SortableTuple> finishList = new ArrayList<>();
		for ( Term kvp : finish ) {
			finishList.add( new SortableTuple(kvp.getKey(), kvp.getValue().getIntValue()) );
		}
		Collections.sort(finishList);
		
		LockableList V_sorted = new LockableList();
		for ( int i = finishList.size()-1; i >= 0 ; i-- ) {
			V_sorted.add(finishList.get(i).object);
		}
		
		Term trans_result = trans.apply(Term.tuple(
				Term.keyVal(GraphTerm.Nodes, Term.list(V_sorted)), 
				Term.keyVal(GraphTerm.Edges, G.getGraphTerm().get(GraphTerm.Edges))));
		TupleTerm G_trans = trans_result.asTuple();
			
		Term dfs_inv_result = DFS.apply(G_trans);
				
		return dfs_inv_result.get(DepthFirstSearch.ComponentKey);
	}

	/**
	 * Tuple of Integer and generic object that overrides the compareTo function
	 * to allow sorting based on Integer value.
	 * @author Uwe Koeckemann
	 *
	 */
	class SortableTuple implements Comparable<SortableTuple> {
		public Integer value;
		public Term object;
		
		public SortableTuple( Term t, Integer value ) {
			this.value = value;
			this.object = t;
		}
		
		@Override
		public int compareTo( SortableTuple a )  {
			return this.value.compareTo(a.value);
		}
		
		@Override
		public String toString() {
			return value.toString() + ": " + object.toString();
		}
	}
}
