package org.aiddl.common.java.math.graph;

import java.util.LinkedList;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class PathExtractor implements Function, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.math.graph.path-extractor");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply(Term args) {
		Term predecessors = args.get(0);
		Term start = args.get(1);
		Term goal = args.get(2);

		if ( predecessors.equals(CommonTerm.NIL) ) {
			return CommonTerm.NIL;
		} else if ( start.equals(goal) ) {
			return Term.list();
		} else if ( predecessors.get(goal).equals(CommonTerm.NIL) ) {
			return CommonTerm.NIL;
		}
		
		LinkedList<Term> path = new LinkedList<>();
		Term current = goal;
		while ( !predecessors.get(current).equals(start) ) {
			path.addFirst(current);
			current = predecessors.get(current);
		}
		path.addFirst(current);
		path.addFirst(start);
			
		return Term.list(path);
	}

}
