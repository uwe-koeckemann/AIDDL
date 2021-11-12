package org.aiddl.common.java.learning.testing;

import java.util.Map;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class AccuracyCalculator implements ConfigurableFunction {
	
	int verbose = 0;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}

	@Override
	public Term apply( Term confusionMatrix ) {
		TupleTerm M = confusionMatrix.get(LearningTerm.Matrix).asTuple();
		
		int good = 0;
		int bad  = 0;
		
		for ( int i = 0 ; i < M.size() ; i++ ) {
			for(  int j = 0 ; j < M.get(i).size() ; j++ ) {
				if ( i == j ) {
					good += M.get(i).get(j).getIntValue();
				} else {
					bad += M.get(i).get(j).getIntValue();
				}
			}
		}
		
		double acc = ((double)good) / ((double)(good+bad));
		
		if ( verbose >= 1 ) {
			Logger.msg("AccuracyCalculator", "Accuracy: " + acc + " from " + confusionMatrix);
		}
		
		return Term.real(acc);
	}	
}
