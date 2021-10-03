package org.aiddl.common.optimization.combinatorial;

import org.aiddl.common.optimization.combinatorial.knapsack.KnapsackGenerator;

import org.aiddl.core.representation.Term;

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
