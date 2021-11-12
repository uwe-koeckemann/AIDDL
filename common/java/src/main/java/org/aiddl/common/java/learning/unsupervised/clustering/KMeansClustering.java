package org.aiddl.common.java.learning.unsupervised.clustering;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.math.linear_algebra.NormL;
import org.aiddl.common.java.math.linear_algebra.VectorAddition;
import org.aiddl.common.java.math.linear_algebra.VectorScalarMultiplication;
import org.aiddl.common.java.math.linear_algebra.VectorSubtraction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.IntegerTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class KMeansClustering implements Function {

	@Override
	public Term apply(Term args) {
		IntegerTerm k = args.get(0).asInt();
		
		CollectionTerm data = args.get(1).asCollection();
		
		List<Term> m = new ArrayList<>();
		
		VectorScalarMultiplication scalarMult = new VectorScalarMultiplication();
		VectorAddition add = new VectorAddition();
		VectorSubtraction sub = new VectorSubtraction();
		NormL l2norm = new NormL(2);
		
		Map<Term, Integer> assignment = new HashMap<>();
		
		/**
		 * "Random" Initialize
		 */
		for ( Term e : data ) {
			m.add(e);
			if ( m.size() == k.getIntValue() ) {
				break;
			}
		}
		
		List<Set<Term>> S = new ArrayList<>();
		for ( int i = 0 ; i < k.getIntValue() ; i++ ) {
			S.add(new HashSet<>());
		}
		
		boolean change = true;
		while ( change ) {
			change = false;
			/**
			 * Assign
			 */
			for ( Term e : data ) {
				int argMin = -1;
				NumericalTerm min = null;
				for ( int i = 0 ; i < m.size() ; i++ ) {
					Term m_i = m.get(i);
					NumericalTerm d = l2norm.apply(sub.apply(Term.tuple(e, m_i))).asNum();
//					System.out.println(d);
//					System.out.println(min);
					if ( min == null || d.lessThan(min) ) {
						argMin = i;
						min = d;
					}
				}
				if ( !change && !assignment.getOrDefault(e, -1).equals(argMin) ) {
					change = true;
				}
				assignment.put(e, argMin);
				S.get(argMin).add(e);
			}
			/**
			 * Update
			 */
			for ( int i = 0 ; i < m.size() ; i++ ) {
				Term s = null;
				for ( Term e : S.get(i) ) {
					if ( s == null ) {
						s = e;
					} else {
						s = add.apply(Term.tuple(s, e));
					}
				}
				double div = 1.0/((double)S.get(i).size());
				s = scalarMult.apply(Term.tuple(Term.real(div), s));
				m.set(i, s);
			}
		}
		return Term.tuple(m);
	}
}
