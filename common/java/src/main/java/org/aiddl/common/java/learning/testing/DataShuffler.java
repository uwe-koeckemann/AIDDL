package org.aiddl.common.java.learning.testing;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class DataShuffler implements ConfigurableFunction {

	Random rng = new Random();
	
	String name = "ShuffleData";
	int verbose = 0;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		
	}
	
	@Override
	public Term apply( Term data ) {	
		if ( verbose >= 1 ) {
			Logger.msg(name, "Shuffling data...");
		}
		
		List<Term> shuffle = new ArrayList<>();
		data.asList().addAllTo(shuffle);
//		shuffle.addAll(data.getList());
		for ( int i = 0 ; i < data.size() ; i++ ) {
			for ( int j = 0 ; j < data.size() ; j++ ) {
				if ( i != j ) {
					if ( rng.nextBoolean() ) {
						Term d_i = shuffle.get(i);
						shuffle.set(i, shuffle.get(j));
						shuffle.set(j, d_i);	
					}
				}
			}
		}
		return Term.list(shuffle);
	}
}
