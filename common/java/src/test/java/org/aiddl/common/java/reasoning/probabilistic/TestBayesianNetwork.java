package org.aiddl.common.java.reasoning.probabilistic;

import org.aiddl.common.java.reasoning.probabilistic.bayesian_networks.McmcSampler;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestBayesianNetwork extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	public void testBayesianNetwork01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
//		Parser.setVerbose(true);
		Parser.parseFile(aiddlTestStr + "/reasoning/probabilistic/bayesian-network.aiddl", db, fReg);

		McmcSampler mcmc = new McmcSampler(1000);

		Term query = db.getEntry(Term.sym("query-01")).getValue().resolve(db);

		Term answer = mcmc.apply(query);

		double n = Double.valueOf(answer.get(Term.sym("T")).getNumerator());
		double d = Double.valueOf(answer.get(Term.sym("T")).getDenominator());

		double epsilon = 0.15;
		assertTrue( Math.abs(n/d - 0.6) < epsilon );
	}

	public void testBayesianNetwork02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		String aiddlTestStr = "../test/";
		Parser.parseFile(aiddlTestStr + "/reasoning/probabilistic/bayesian-network.aiddl", db, fReg);

		McmcSampler mcmc = new McmcSampler(1000);

		Term query = db.getEntry(Term.sym("query-02")).getValue().resolve(db);

		Term answer = mcmc.apply(query);

		double n = Double.valueOf(answer.get(Term.sym("T")).getNumerator());
		double d = Double.valueOf(answer.get(Term.sym("T")).getDenominator());

		double epsilon = 0.05;
		assertTrue( Math.abs(n/d - 0.5) < epsilon );
	}

	public void testBayesianNetwork03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		String aiddlTestStr = "../test/";
//		Parser.setVerbose(true);
		Parser.parseFile(aiddlTestStr + "/reasoning/probabilistic/bayesian-network.aiddl", db, fReg);

		McmcSampler mcmc = new McmcSampler(1000);

		Term query = db.getEntry(Term.sym("query-03")).getValue().resolve(db);

		Term answer = mcmc.apply(query);

		double n = Double.valueOf(answer.get(Term.sym("T")).getNumerator());
		double d = Double.valueOf(answer.get(Term.sym("T")).getDenominator());

		double epsilon = 0.05;
		assertTrue( Math.abs(n/d - 0.0) < epsilon );
	}
}
