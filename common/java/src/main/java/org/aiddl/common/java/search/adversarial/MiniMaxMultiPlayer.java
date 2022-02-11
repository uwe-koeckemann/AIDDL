package org.aiddl.common.java.search.adversarial;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;

public class MiniMaxMultiPlayer implements Function {
	
	Function f_score;
	Function f_game_over;
	Function f_trans;
	Function f_applicable;
	
	int minMaxCalls = 0;
	int cacheFinds = 0;
	int cacheAdds = 0;
	int leaves = 0;
	
	Map<Term, Term> cache = new HashMap<>();
	
	RelativeScore relativeScore = new RelativeScore();
	
	
	public MiniMaxMultiPlayer( Function f_score, Function f_game_over, Function f_trans, Function f_applicable ) {
		this.f_score = f_score;
		this.f_game_over = f_game_over;
		this.f_trans = f_trans;
		this.f_applicable = f_applicable;
	}

	@Override
	public Term apply(Term arg0) {
		Term state = arg0.get(0);
		SetTerm actions = arg0.get(1).asSet();
		
		minMaxCalls = 0;
		cacheFinds = 0;
		cacheAdds = 0;
		Term pair = maxValue(state, actions);		
//		System.out.println("Number of internal calls: " + minMaxCalls);
//		System.out.println("Number of cache finds: " + cacheFinds);
//		System.out.println("Number of cache adds: " + cacheAdds);
//		System.out.println("Number of leaf nodes: " + leaves);
//		System.out.println("Final score: " + pair.get(0));
		
		return pair.get(1);
	}

	private Term maxValue( Term state, SetTerm actions ) {
		if ( cache.containsKey(state) ) {
			cacheFinds++;
			return cache.get(state);
		}
		minMaxCalls++;
//		System.out.println("MAX: " + state);
		NumericalTerm max = Term.infNeg();
		if ( f_game_over.apply(state).getBooleanValue() ) {
			leaves++;
//			System.out.println("MAX LEAF: " + state);
//			System.out.println("SCORE: " + f_score.compute(state));
			return Term.tuple(f_score.apply(state), CommonTerm.NIL);
		}
		Term argMax = null;
		Term argMaxScoreList = null;
		
		Term current_player = state.get(Term.sym("next-player"));
		
		for ( Term action : f_applicable.apply(Term.tuple(state, actions)).asCollection() ) {
			Term next_state = f_trans.apply(Term.tuple(state, action));
			if ( !next_state.equals(CommonTerm.NIL) ) {
				Term pair = maxValue(next_state, actions);
				NumericalTerm v = relativeScore.apply(Term.tuple(current_player, pair.get(0))).asNum();
				if ( v.greaterThan(max) ) {
					max = v;
					argMax = action;
					argMaxScoreList = pair.get(0);
				}
			}
		}
		
		cache.put(state, Term.tuple(argMaxScoreList, argMax));
		cacheAdds++;
		return Term.tuple(argMaxScoreList, argMax);
	}
}
