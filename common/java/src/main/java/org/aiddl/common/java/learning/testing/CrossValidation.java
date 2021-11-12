package org.aiddl.common.java.learning.testing;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.FunctionGenerator;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.IntegerTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

/**
 * Run cross validation on a data set and return list of real and predicted value pairs.
 * 
 * @author Uwe Koeckemann
 *
 */
public class CrossValidation implements Function {
	
	private static final Term NumFolds = Term.sym("num-folds");

	@Override
	public Term apply(Term args) {
		Term data = args.get(LearningTerm.Data);
		Term attributes = args.get(LearningTerm.Attributes);
		Term label = args.get(LearningTerm.Label);
		FunctionReferenceTerm learner = args.get(LearningTerm.Learner).asFunRef();
		
		if ( !(learner.getFunction() instanceof FunctionGenerator) ) {
			throw new IllegalArgumentException("Learner does not implement FunctionGenerator (used to get predictor function):\n" + learner);
		}

		IntegerTerm num_folds = args.getOrDefault(NumFolds, Term.integer(10)).asInt();
		
		int label_idx = 0;
		for ( int i = 0 ; i < attributes.size() ; i++ ) {
			if ( attributes.get(i).get(0).equals(label) ) {
				label_idx = i;
				break;
			}
		}
		
		DataShuffler shuffle = new DataShuffler();
		DataSplitter splitter = new DataSplitter();

		data = shuffle.apply(data);
		
		LockableList results = new LockableList();
		
		for ( int i = 0 ; i < num_folds.getIntValue() ; i++ ) {
			Term split = splitter.apply(Term.tuple(
					Term.keyVal(LearningTerm.Fold, Term.integer(i)),
					Term.keyVal(LearningTerm.Data, data)
					));
					
			Term train = split.get(LearningTerm.TrainData);
			Term test  = split.get(LearningTerm.TestData);
			
			learner.getFunction().apply(
					Term.tuple(
							Term.keyVal(LearningTerm.Attributes, args.get(LearningTerm.Attributes)),
							Term.keyVal(LearningTerm.Label, args.get(LearningTerm.Label)),
							Term.keyVal(LearningTerm.Data, train)
							)
					);
			Function predictor = ((FunctionGenerator)learner.getFunction()).generate();
			
			for ( int j = 0 ; j < test.size() ; j++ ) {
				Term y_true = test.get(j).get(label_idx);
				Term y_pred = predictor.apply(test.get(j));
				results.add(Term.tuple(y_true, y_pred));
			}
		}
		
		return Term.list(results);
	}
}
