package org.aiddl.common.learning.supervised.classification;

import org.aiddl.common.learning.decision_tree.DecisionTreeClassifier;
import org.aiddl.common.learning.decision_tree.ID3;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestDecisionTreeLearning extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
		
	public void testPlayTennis() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		String aiddlTestStr = "../test/";
		Parser.parseFile(aiddlTestStr + "/learning/classification/problem-01.aiddl", db, fReg);
	
		ID3 id3 = new ID3();
		Term mlPRoblem = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
	
		Term decisionTree = id3.apply(mlPRoblem);

		Term atts = db.getEntry(Term.sym("WeatherAttributes")).getValue();
		
		DecisionTreeClassifier classify = new DecisionTreeClassifier();
		
		classify.initialize(Term.tuple(decisionTree, atts));	
					
		assertTrue( Term.sym("Yes").equals( classify.apply(Parser.ParseTerm("[Sunny Cool Normal Weak ?A]"))) );
	}
}
