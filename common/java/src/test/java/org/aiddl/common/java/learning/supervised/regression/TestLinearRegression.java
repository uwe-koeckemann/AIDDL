package org.aiddl.common.java.learning.supervised.regression;

import java.util.HashMap;
import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.common.java.learning.linear_regression.LinearRegression;
import org.aiddl.common.java.learning.linear_regression.OrdinaryLeastSquaresRegression;
import org.aiddl.common.java.learning.testing.CrossValidation;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;

import junit.framework.TestCase;
import org.aiddl.util.java.function.RegistryLoader;

@SuppressWarnings("javadoc")
public class TestLinearRegression extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testProblemType() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/learning/regression/problem-01.aiddl", db, fReg).asSym();
		
		Entry problemEntry = db.getEntry(Term.sym("problem"));
		
		Term problem = problemEntry.getValue();
		Function typeCheck = problemEntry.getType().asFunRef().getFunction();
		
		Term r = typeCheck.apply(problem);
		
		assertTrue(r.getBooleanValue());
	}
	
	
	public void testOneDimensionalWithNoise() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		RegistryLoader.register(fReg, db);
		
		SymbolicTerm name = Parser.parseFile("../test/learning/regression/line-regression.aiddl", db, fReg).asSym();
		
		Function f = fReg.getFunction(name.concat(Term.sym("f")));
		
		f.apply(Term.real(1.0));
		
		Function generator = fReg.getFunction(name.concat(Term.sym("generate")));
//		Term generate = db.getEntry(Term.sym("generate")).getValue();
				
		SetTerm data = generator.apply(Term.integer(50)).asSet();
		
		LockableList X_l = new LockableList();
		LockableList y_l = new LockableList();
		
		for ( Term point : data ) {
			y_l.add(point.get(0));
			LockableList x = new LockableList();
			for ( int i = 1 ; i < point.size() ; i++ ) {
				x.add(point.get(i));
			}
			X_l.add(Term.tuple(x));
		}
		
		Term X = Term.tuple(X_l);
		Term y = Term.tuple(y_l);
		OrdinaryLeastSquaresRegression oReg = new OrdinaryLeastSquaresRegression();
		Term w_oreg = oReg.apply(Term.tuple(X, y));
	}
	
	public void testCrossValidation() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/learning/regression/problem-01.aiddl", db, fReg).asSym();
		
		Entry problemEntry = db.getEntry(Term.sym("problem"));
		
		Term problem = problemEntry.getValue();
		
		CrossValidation cval = new CrossValidation();
		
		LinearRegression lreg = new LinearRegression();
		lreg.configure(new HashMap<Term, Term>(), fReg);
		fReg.addFunction(Term.sym("learner"), lreg);
		FunctionReferenceTerm lreg_fun = Term.fref(Term.sym("learner"), fReg);
		
		Term results = cval.apply(
				Term.tuple(
						Term.keyVal(LearningTerm.Attributes, problem.get(LearningTerm.Attributes)),
						Term.keyVal(LearningTerm.Label, problem.get(LearningTerm.Label)),
						Term.keyVal(LearningTerm.Data, problem.get(LearningTerm.Data)),
						Term.keyVal(LearningTerm.Learner, lreg_fun)
						)
				);
		
		assertTrue( results.size() == 50 );
	}
}