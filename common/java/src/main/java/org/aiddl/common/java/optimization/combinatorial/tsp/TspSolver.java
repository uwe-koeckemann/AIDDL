package org.aiddl.common.java.optimization.combinatorial.tsp;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.math.graph.GraphTerm;
import org.aiddl.common.java.search.TreeSearch;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class TspSolver implements Function {

	@Override
	public Term apply(Term tsp) {
		TspMinRemainder remainder = new TspMinRemainder();
		remainder.initialize(tsp);
		
		TspExpandPath expand = new TspExpandPath();
		expand.initialize(tsp);
		
		TreeSearch search = new TreeSearch();
			
		Term path = Term.list();
		
		NumericalTerm best = Term.infPos();
		Term argBest = null;
		
		int numNodes = tsp.get(GraphTerm.Nodes).size();
		
		int a_count = 0;
		
		while ( !path.equals(CommonTerm.NIL) ) {
			Term expansion = expand.apply(path);
			a_count += 1;
			path = search.apply(Term.tuple(Term.sym("expand"), expansion));
			if ( !path.equals(CommonTerm.NIL) ) {
				NumericalTerm pathCost = this.pathCost(path.asList());
				pathCost = remainder.apply(path).asNum().add(pathCost);

				while (!path.equals(CommonTerm.NIL) && pathCost.greaterThan(best)) {
					path = search.apply(Term.tuple(Term.sym("next")));
					if (!path.equals(CommonTerm.NIL)) {
						pathCost = this.pathCost(path.asList());
						pathCost = remainder.apply(path).asNum().add(pathCost);
					}
					a_count += 1;
				}
				if ( !path.equals(CommonTerm.NIL) && path.size() == numNodes && pathCost.lessThan(best)) {
					best = pathCost;
					argBest = path;
				}
			}
		}
		
		LockableList r = new LockableList();
		for ( Term e : argBest.asList() ) {
			r.add(e.get(1).getKey());
		}
		return Term.tuple(
				Term.keyVal(Term.sym("path"), Term.list(r)),
				Term.keyVal(Term.sym("cost"), best));

	}

	
	public NumericalTerm pathCost( ListTerm path ) {
		NumericalTerm sum = Term.integer(0);
		for ( Term e : path ) {
			sum = sum.add(e.get(0).asNum() );
		}
		return sum;
	}
}
