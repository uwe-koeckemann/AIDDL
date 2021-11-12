package org.aiddl.common.java.math.graph;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.KeyValueTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;

public class Transpose implements Function, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.math.graph.transpose");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply( Term G ) {
		Term V = G.get(GraphTerm.Nodes);
		SetTerm E = G.get(GraphTerm.Edges).asSet();
		
		LockableSet E_trans = new LockableSet();
		for ( Term e : E ) {
			Term new_edge = Term.tuple(e.get(1), e.get(0));
			E_trans.add(new_edge);
		}
		
		LockableList G_new = new LockableList();
		for ( int i = 0 ; i < G.size() ; i++ ) {
			if ( G.get(i) instanceof KeyValueTerm ) {
				if ( !G.get(i).getKey().equals(GraphTerm.Edges) ) {
					G_new.add(G.get(i));
				}
			} else {
				G_new.add(G.get(i));
			}
		}
		
		return Term.tuple(
				Term.keyVal(GraphTerm.Nodes, V), 
				Term.keyVal(GraphTerm.Edges, Term.set(E_trans)));
	}
}
