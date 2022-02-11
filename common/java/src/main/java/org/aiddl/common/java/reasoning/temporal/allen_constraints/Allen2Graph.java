package org.aiddl.common.java.reasoning.temporal.allen_constraints;

import java.util.HashSet;

import org.aiddl.common.java.math.graph.GraphTerm;
import org.aiddl.common.java.reasoning.temporal.TemporalTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;

public class Allen2Graph implements Function {
	
	private HashSet<Term> unaryConstraints = new HashSet<>();
	
	public Allen2Graph() {
		unaryConstraints.add(TemporalTerm.Release);
		unaryConstraints.add(TemporalTerm.Deadline);
		unaryConstraints.add(TemporalTerm.Duration);
		unaryConstraints.add(TemporalTerm.At);
	}
	
	@Override
	public Term apply(Term AIC) {
		LockableSet V = new LockableSet();
		LockableSet E = new LockableSet();
		LockableSet L = new LockableSet();
		
		for ( Term e : AIC.asCollection() ) {
			Term con = e.get(0);
			if ( unaryConstraints.contains(con) ) {
				Term v1 = e.get(1); 
				V.add(v1);
				LockableList label = new LockableList();
				label.add(con);
				label.add(e.get(2));
				if ( e.size() > 3 ) {
					label.add(e.get(3));
				}
				Term edge = Term.tuple(v1, v1); 
				E.add(edge);
				L.add(Term.keyVal(edge, Term.tuple(label)));
			} else {
				Term v1 = e.get(1); 
				Term v2 = e.get(2);
				V.add(v1);
				V.add(v2);
				LockableList label = new LockableList();
				label.add(con);
				if ( e.size() > 3 ) {
					label.add(e.get(3));
					if ( e.size() > 4 ) {
						label.add(e.get(4));
					}
				}
				Term edge = Term.tuple(v1, v2); 
				E.add(edge);
				L.add(Term.keyVal(edge, Term.tuple(label)));
			}
		}
		
		return Term.tuple(
				Term.keyVal(GraphTerm.Nodes, Term.set(V)),
				Term.keyVal(GraphTerm.Edges, Term.set(E)),
				Term.keyVal(GraphTerm.Labels, Term.set(L)));
	}

}
