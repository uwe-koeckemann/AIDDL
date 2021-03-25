package org.aiddl.common.math.graph;

import org.aiddl.core.representation.Term;

public class GraphTools {
		
	public static Term assembleGraph ( Term V, Term E ) {
		return Term.tuple(Term.keyVal(GraphTerm.Nodes, V), Term.keyVal(GraphTerm.Edges, E));
	}
}
