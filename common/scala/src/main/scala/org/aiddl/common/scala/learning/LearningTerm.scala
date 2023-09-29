package org.aiddl.common.scala.learning

import org.aiddl.core.scala.representation.Sym

enum Term(val sym: Sym):
	case Fit extends Term(Sym("fit"))
	case Predict extends Term(Sym("predict"))
	case SetParams extends Term(Sym("set-params"))



object LearningTerm {
	val Learner 			= Sym("learner")
	val Predictor			= Sym("predictor")
	val Classifier			= Sym("classifier")
	val Class			= Sym("class")
	
	val Data 				= Sym("data")
	val Attributes 		    = Sym("attributes")
	val Instance 			= Sym("instance")
	val Label 				= Sym("label")

	val ConfusionMatrix 	= Sym("confusion-matrix")
	val Matrix				= Sym("matrix")
	
	val Fold 				= Sym("fold")
	
	val TrainData 			= Sym("train-data")
	val TestData 			= Sym("test-data")
}