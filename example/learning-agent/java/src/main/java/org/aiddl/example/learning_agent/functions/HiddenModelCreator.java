package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;

public class HiddenModelCreator implements Function {

	Random rng = new Random();
	
	@Override
	public Term apply( Term args ) {
		CollectionTerm locations = (CollectionTerm) args.get(Term.sym("locations"));
		CollectionTerm configs = (CollectionTerm) args.get(Term.sym("configs"));
		Term label = args.get(LearningTerm.Label);
		
		List<Term> map = new ArrayList<>();
		for ( Term l : locations ) {
			for ( Term c : configs ) {
				map.add(Term.keyVal(Term.tuple(l, c), label.get(rng.nextInt(label.size()))));
			}
		}

		return Term.list(map);
	}
}
