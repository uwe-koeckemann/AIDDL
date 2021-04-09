package org.aiddl.common.optimization.combinatorial.tsp;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.IntegerTerm;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

/**
 * Generate a random Traveling Salesperson Problem (TSP).
 * @author Uwe Koeckemann
 *
 */
public class TspGenerator implements Function {

	@Override
	public Term apply( Term args ) {
		IntegerTerm n = args.get(0).asInt();
		IntegerTerm x_max = args.get(1).asInt();
		IntegerTerm y_max = args.get(2).asInt();
		
		Random r = new Random(1001);
		
		LockableSet coordinates = new LockableSet();
		for ( int i = 0 ; i < n.getIntValue(); i++ ) {
			Term n_i = Term.sym(String.format("n%d", (i+1)));
			Term pos = Term.tuple(
								Term.integer(r.nextInt(x_max.getIntValue()+1)),	 
								Term.integer(r.nextInt(y_max.getIntValue()+1)));	
			coordinates.add(Term.keyVal(n_i, Term.set(Term.keyVal(Term.sym("pos"), pos))));
		}
		
		LockableSet nodes = new LockableSet();
		LockableSet edges = new LockableSet();
		LockableSet weights = new LockableSet();
		
		for ( int i = 0; i < n.getIntValue() ; i++ ) {
			Term n_i = Term.sym(String.format("n%d", i+1)); 
			nodes.add(n_i);
			for ( int j = 0; j < n.getIntValue() ; j++ ) {
				if ( i != j ) {
					Term n_j =Term.sym(String.format("n%d", j+1));
					
					Double x_i = Double.valueOf(coordinates.get(n_j).get(Term.sym("pos")).get(0).getIntValue());
					Double y_i = Double.valueOf(coordinates.get(n_i).get(Term.sym("pos")).get(1).getIntValue());
					
					Double x_j = Double.valueOf(coordinates.get(n_j).get(Term.sym("pos")).get(0).getIntValue());
					Double y_j = Double.valueOf(coordinates.get(n_j).get(Term.sym("pos")).get(1).getIntValue());
					
					int distance = (int)(Math.sqrt(Math.pow(Math.abs(x_i-x_j), 2.0) + Math.pow(Math.abs(y_i-y_j), 2.0)));
					
					List<KeyValueTerm> L = new ArrayList<>();
					Term e = Term.set(n_i, n_j);
					edges.add(e);
					weights.add(Term.keyVal(e, Term.integer(distance)));
				}
			}
		}
		return Term.tuple(
				Term.keyVal(GraphTerm.Nodes, Term.set(nodes)),
				Term.keyVal(GraphTerm.Edges, Term.set(edges)),
				Term.keyVal(GraphTerm.Weights, Term.set(weights)),
				Term.keyVal(GraphTerm.Attributes, Term.set(coordinates) ));
	}


}
