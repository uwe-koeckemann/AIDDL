package org.aiddl.common.optimization.combinatorial.knapsack;

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
public class KnapsackGenerator implements Function {

	@Override
	public Term apply( Term args ) {
		IntegerTerm weight_limit = args.get(0).asInt();
		IntegerTerm num_items = args.get(1).asInt();
		IntegerTerm item_num_max = args.get(2).asInt();
		IntegerTerm item_value_max = args.get(3).asInt();		
		IntegerTerm item_weight_max = args.get(4).asInt();
		
		Random r = new Random(1001);
		
		LockableSet items = new LockableSet();
		
		for ( int i = 0 ; i < num_items.getIntValue(); i++ ) {
			IntegerTerm value = Term.integer(1 + r.nextInt(item_value_max.getIntValue()));
			IntegerTerm weight = Term.integer(1 + r.nextInt(item_weight_max.getIntValue()));
			
			items.add(Term.tuple(Term.sym("i" + (i+1)), value, weight));
		}

		return Term.tuple(
				weight_limit,
				item_num_max,
				Term.set(items)
				);
	}


}
