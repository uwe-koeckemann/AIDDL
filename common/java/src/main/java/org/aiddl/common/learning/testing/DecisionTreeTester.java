package org.aiddl.common.learning.testing;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.common.learning.decision_tree.DecisionTreeClassifier;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.IntegerTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class DecisionTreeTester implements ConfigurableFunction {

	int verbose = 0;
	String name = "TestModel";
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Term apply( Term args ) {
		Term decisionTree = args.get(Term.sym("dtree"));
		Term atts  = args.get(LearningTerm.Attributes);
		ListTerm data  = (ListTerm) args.get(LearningTerm.Data);
		Term cMatrix  = args.get(LearningTerm.Matrix);
		
		Term cMatrixValues  = cMatrix.get(LearningTerm.Matrix);
		ListTerm cMatrixAtts = (ListTerm) cMatrix.get(LearningTerm.Attributes);
		List<List<NumericalTerm>> cMatrixNew = new ArrayList<>();
		for ( int i = 0 ; i < cMatrixAtts.size() ; i++ ) {
			List<NumericalTerm> row = new ArrayList<>();
			for ( int j = 0 ; j < cMatrixAtts.size() ; j++ ) {
				row.add((IntegerTerm) cMatrixValues.get(i).get(j));
			}
			cMatrixNew.add(row);
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "Testing: " + decisionTree);
			Logger.incDepth();
		}
		
		DecisionTreeClassifier dtClass = new DecisionTreeClassifier();
		dtClass.initialize(Term.tuple(decisionTree, atts));
		
		for ( Term dp : data ) {			
			Term dtAnswer = dtClass.apply(dp);
			Term realAnswer = dp.get(2);
			
			int i = cMatrixAtts.indexOf(dtAnswer);
			int j = cMatrixAtts.indexOf(realAnswer);
			
			if ( i != -1 ) {
				cMatrixNew.get(i).set(j, cMatrixNew.get(i).get(j).add(Term.integer(1)) );
				if ( verbose >= 2 ) Logger.msg(name, "Correctly classied: " + dp);
			} else {
				if ( verbose >= 2 ) Logger.msg(name, "Failed to classify: " + dp);
			}
		}

		
		List<TupleTerm> cReturnRows = new ArrayList<>();
		for ( int i = 0 ; i < cMatrixNew.size() ; i++ ) {
			cReturnRows.add(Term.tuple(cMatrixNew.get(i)));
		}
		Term cMatrixUpdated = Term.tuple(
				Term.keyVal(LearningTerm.Attributes, cMatrixAtts), 
				Term.keyVal(LearningTerm.Matrix, Term.tuple(cReturnRows)));
		
		if ( verbose >= 1 ) {
			Logger.decDepth();
			Logger.msg(name, "Results: " + cMatrixUpdated);
		}		
		return cMatrixUpdated;
	}
}
