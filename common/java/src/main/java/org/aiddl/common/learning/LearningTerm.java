package org.aiddl.common.learning;

import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class LearningTerm {
	public static final SymbolicTerm Learner 			= Term.sym("learner");
	public static final SymbolicTerm Predictor			= Term.sym("predictor");
	public static final SymbolicTerm Classifier			= Term.sym("classifier");
	
	public static final SymbolicTerm Data 				= Term.sym("data");
	public static final SymbolicTerm Attributes 		= Term.sym("attributes");
	public static final SymbolicTerm Instance 			= Term.sym("instance");
	public static final SymbolicTerm Label 				= Term.sym("label");

	public static final SymbolicTerm ConfusionMatrix 	= Term.sym("confusion-matrix");
	public static final SymbolicTerm Matrix				= Term.sym("matrix");
	
	public static final SymbolicTerm Fold 				= Term.sym("fold");
	
	public static final SymbolicTerm TrainData 			= Term.sym("train-data");
	public static final SymbolicTerm TestData 			= Term.sym("test-data");
}
