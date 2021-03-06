package org.aiddl.common.reasoning.probabilistic;

import org.aiddl.common.reasoning.probabilistic.bayesian_networks.McmcSampler;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestBayesianNetwork extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	public void testBayesianNetwork() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
//		Parser.setVerbose(true);
		Parser.parseFile(aiddlTestStr + "/reasoning/probabilistic/bayesian-network.aiddl", db, fReg);
		
		McmcSampler mcmc = new McmcSampler(10000);
		
		Term query = db.getEntry(Term.sym("query")).getValue().resolve(db);
	
		Term answer = mcmc.apply(query);
		
		double n = Double.valueOf(answer.get(Term.sym("T")).getNumerator());
		double d = Double.valueOf(answer.get(Term.sym("T")).getDenominator());
		
		double epsilon = 0.05;
		assertTrue( Math.abs(n/d - 0.5) < epsilon );
	}
}
