package org.aiddl.common.learning.testing;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;

public class DataSplitter implements ConfigurableFunction {
	
	int numFolds = 2;
	
	int verbose = 0;
	String name = "SplitData";

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.numFolds = settings.getOrDefault(Term.sym("num-folds"), Term.integer(2)).getIntValue();
	}

	@Override
	public Term apply( Term args ) {
		int test_fold = args.get(LearningTerm.Fold).getIntValue();
		ListTerm data  = (ListTerm) args.get(LearningTerm.Data);
		
		int foldSize = data.size() / numFolds;
				
		if ( verbose >= 1 ) {
			Logger.msg(name, "Splitting fold " + test_fold + "/" + numFolds + " fold size: " + foldSize );
		}
		
		
		List<Term> trainingData = new ArrayList<>();
		List<Term> testData = new ArrayList<>();
		for ( int i = 0 ; i < data.size() ; i++ ) {
			int current_fold = i%numFolds;
			
			if ( current_fold == (test_fold-1) ) {
				testData.add(data.get(i));
			} else {
				trainingData.add(data.get(i));
			}
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "|train| = " + trainingData.size());
			Logger.msg(name, "|test|  = " + testData.size());
		}
		
		LockableList rList = new LockableList();
		rList.add(Term.keyVal(LearningTerm.TrainData, Term.list(trainingData)));
		rList.add(Term.keyVal(LearningTerm.TestData, Term.list(testData)));
		return Term.list(rList);
	}
	

}
