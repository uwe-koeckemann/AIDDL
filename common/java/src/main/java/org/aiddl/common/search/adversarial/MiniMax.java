package org.aiddl.common.search.adversarial;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;

public class MiniMax implements Function {
	
	Function f_score;
	Function f_game_over;
	Function f_trans;
	Function f_applicable;
	
	int minMaxCalls = 0;
	int cacheFinds = 0;
	int cacheAdds = 0;
	int leaves = 0;
	
	Map<Term, Term> cache = new HashMap<>();
	
	
	public MiniMax( Function f_score, Function f_game_over, Function f_trans, Function f_applicable ) {
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
		
		Term current_player = state.get(Term.sym("next-player"));
		
		for ( Term action : f_applicable.apply(Term.tuple(state, actions)).asCollection() ) {
			Term next_state = f_trans.apply(Term.tuple(state, action));
			if ( !next_state.equals(CommonTerm.NIL) ) {
				Term pair = minValue(next_state, actions);
				NumericalTerm v = pair.get(0).asNum();
				if ( v.greaterThan(max) ) {
					max = v;
					argMax = action;
				}
			}
		}
		
		cache.put(state, Term.tuple(max, argMax));
		cacheAdds++;
		return Term.tuple(max, argMax);
	}
	
	private Term minValue( Term state, SetTerm actions ) {
		if ( cache.containsKey(state) ) {
			cacheFinds++;
			return cache.get(state);
		}
		minMaxCalls++;
//		System.out.println("MIN: " + state);
		
		NumericalTerm min = Term.infPos();
		if ( f_game_over.apply(state).getBooleanValue() ) {
			leaves++;
//			System.out.println("MIN LEAF: " + state);
//			System.out.println("SCORE: " + f_score.compute(state));
			return Term.tuple(f_score.apply(state), CommonTerm.NIL);
		}
		Term argMin = null;
		
		for ( Term action : f_applicable.apply(Term.tuple(state, actions)).asCollection() ) {
			Term next_state = f_trans.apply(Term.tuple(state, action));
			if ( !next_state.equals(CommonTerm.NIL) ) {
				Term pair = maxValue(next_state, actions);
				NumericalTerm v = pair.get(0).asNum();
				if ( v.lessThan(min) ) {
					min = v;
					argMin = action;
				}
			}
		}
		cache.put(state, Term.tuple(min, argMin));
		cacheAdds++;
		return Term.tuple(min, argMin);
	}
}
