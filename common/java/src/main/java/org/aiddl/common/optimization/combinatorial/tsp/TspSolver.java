package org.aiddl.common.optimization.combinatorial.tsp;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.common.search.TreeSearch;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

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
			NumericalTerm pathCost = this.pathCost(path.asList());
			pathCost = remainder.apply(path).asNum().add(pathCost);
			
			while ( !path.equals(CommonTerm.NIL) && pathCost.greaterThan(best) ) {
				path = search.apply(Term.tuple(Term.sym("next")));
				if ( !path.equals(CommonTerm.NIL) ) {
					pathCost = this.pathCost(path.asList());
					pathCost = remainder.apply(path).asNum().add(pathCost);
				}
				a_count += 1;
			}
			if ( !path.equals(CommonTerm.NIL) && path.size() == numNodes && pathCost.lessThan(best) ) {
				best = pathCost;
				argBest = path;
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

	
	private NumericalTerm pathCost( ListTerm path ) {
		NumericalTerm sum = Term.integer(0);
		for ( Term e : path ) {
			sum = sum.add(e.get(0).asNum() );
		}
		return sum;
	}
}
