package org.aiddl.common.optimization.combinatorial;

import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.common.math.graph.UndirectedGraph2Dot;
import org.aiddl.common.optimization.combinatorial.knapsack.KnapsackGenerator;
import org.aiddl.common.optimization.combinatorial.tsp.TspGenerator;
import org.aiddl.common.optimization.combinatorial.tsp.TspMinRemainder;
import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.service.RequestHandler;
import org.aiddl.core.tools.Logger;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestKnapsack extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	public void testKnapsackGenerator() {
		KnapsackGenerator knapsackGen = new KnapsackGenerator();
				
		Term problem = knapsackGen.apply(
				Term.tuple(
						Term.integer(50), 
						Term.integer(50), 
						Term.integer(1),
						Term.integer(100),
						Term.integer(10)));
		
		
//		System.out.println(Logger.prettyPrint(problem, 0));
	}
	

}
