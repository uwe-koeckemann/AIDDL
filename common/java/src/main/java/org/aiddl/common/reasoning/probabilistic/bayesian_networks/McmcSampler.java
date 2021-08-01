package org.aiddl.common.reasoning.probabilistic.bayesian_networks;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.RationalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableList.ListType;
import org.aiddl.core.tools.MapType;
import java.util.Random;

public class McmcSampler implements ConfigurableFunction {
	
	private Integer N_samples = 100;
	
	public McmcSampler( Integer N_samples ) {
		this.N_samples = N_samples;
	};
		
	@Override
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg ) {
		N_samples = settings.getOrDefault(Term.sym("samples"), Term.integer(N_samples)).getIntValue();
	}

	@Override
	public Term apply( Term mcmcQuery ) {
		Random r = new Random();

		Term X = mcmcQuery.get(Term.sym("variable"));
		Map<Term,Term> e = ((CollectionTerm)mcmcQuery.get(Term.sym("evidence"))).getMap();
		SetTerm bn = (SetTerm) mcmcQuery.get(Term.sym("bn")); 

		Map<Term,Integer> N = new HashMap<>();
		
		Map<Term,ListTerm> parents = new HashMap<>();
		Map<Term,List<Term>> children = new HashMap<>();
		Map<Term,Map<Term,Term>> CondProb = new HashMap<>();
		Map<Term,ListTerm> values = new HashMap<>();
		
		List<Term> variables = new ArrayList<>();

		for ( Term cpt : bn.asSet() ) {
			Term var = cpt.get(0);
			ListTerm var_parents = (ListTerm) cpt.get(1);
			ListTerm var_values = (ListTerm) cpt.get(2);
			ListTerm table = (ListTerm) cpt.get(3);
			
			variables.add(var);
			values.put(var, var_values);
			parents.put(var, var_parents);
			
			CondProb.put(var, table.getMap());
			
//			if ( var.equals(X) && var_parents.isEmpty() ) {
//				LockableList valueProbs = new LockableList();
//				for ( int i = 0 ; i < var_values.size() ; i++ ) {
//					valueProbs.add(Term.keyVal(var_values.get(i), table.get(0).getValue().get(i)));
//				}
//				return Term.list(valueProbs);
//			}
		
			for ( Term var_parent : var_parents.asList() ) {
				if ( !children.containsKey(var_parent) ) {
					children.put(var_parent, new ArrayList<Term>());
				}
				children.get(var_parent).add(var);
			}
		}
		
		for ( Term x : values.get(X).asList() ) {
			N.put(x, 0);
		}
		
		Set<Term> Z = new LinkedHashSet<Term>();
		for ( Term var : variables ) {
			if ( !e.containsKey(var) ) {
				Z.add(var);
			}
		}
	
		/** 
		 * Init x (fix evidence and randomly choose starting values
		 */
		Map<Term,Term> x = new HashMap<Term,Term>();
		x.putAll(e);
		

		for ( Term z : Z ) {
			ListTerm val_z = values.get(z);
			x.put(z, val_z.get(r.nextInt(val_z.size())));
		}
				
		for ( int i = 1 ; i <= N_samples; i++ ) {
			N.put(x.get(X), N.get(x.get(X))+1);
			for ( Term Z_i : Z ) {
				ListTerm mb_Z_i = getMarkovBedDist(Z_i,  x, values, children, parents, CondProb);

				NumericalTerm roll;
				if ( mb_Z_i.get(0) instanceof RationalTerm ) {
					roll = Term.rational( Long.valueOf(r.nextInt(1000000+1)), 1000000L );
				} else {
					roll = Term.real(r.nextDouble());
				}
				
//				System.out.println("Roll: " +roll);
				
				NumericalTerm sum = Term.rational(0L,1L);
//				Term choice = null;
				sum = Term.rational(0L, 1L);
				for ( int j = 0 ; j < mb_Z_i.size() ; j++ ) {
					sum = sum.add((NumericalTerm) mb_Z_i.get(j));
					if ( roll.lessThan(sum) ) {
//						System.out.println("Choice: " + values.get(Z_i).get(j));
						x.put(Z_i, values.get(Z_i).get(j));
						break;
					}
				}
			}
		}
		
		List<Term> probDist = new ArrayList<>();
		for ( Term k : N.keySet() ) {
			probDist.add(Term.keyVal(k, Term.rational(N.get(k).longValue(), N_samples.longValue())));
		}
		
		return Term.list(probDist);
	}
	
	private ListTerm getMarkovBedDist ( 
			Term X, 
			Map<Term,Term> x, 
			Map<Term,ListTerm> values,
			Map<Term,List<Term>> children,
			Map<Term,ListTerm> parents,
			Map<Term,Map<Term,Term>> P ) {
		
		ListTerm P_X_given_parents = getProbs(X, x, parents.get(X), P.get(X));
		

		
		List<Term> children_X = children.get(X);
		if ( children_X != null && !(children_X.size() == 0 )) {
			List<NumericalTerm> prods = new ArrayList<>();
			Term prev = x.get(X);
			for ( Term v : values.get(X) ) {
				NumericalTerm product = null;
				x.put(X, v);
				for (Term Y_i : children.get(X)) {
					NumericalTerm p_y = getProb(Y_i, x, values.get(Y_i), parents.get(Y_i), P.get(Y_i));
					if (product == null) {
						product = p_y;
					} else {
						product = product.mult(p_y);
					}
				}
				prods.add(product);
			}
			x.put(X, prev);
			
			NumericalTerm sum = Term.rational(0L, 1L);
//			List<NumericalTerm> mb_X = new ArrayList<>();
			LockableList mb_X = new LockableList(ListType.LinkedList, MapType.HashMap);
			
			for ( int i = 0 ; i < P_X_given_parents.size(); i++ ) {
				NumericalTerm p = P_X_given_parents.get(i).asNum();
				mb_X.add(p.mult(prods.get(i)));
				sum = sum.add(p.mult(prods.get(i)));
			}
			for ( int i = 0 ; i < mb_X.size() ; i++ ) {
				if ( !sum.isZero() )
					mb_X.set(i, ((NumericalTerm) mb_X.get(i)).div(sum));
			}
			return Term.list( mb_X ); //.toArray(new NumericalTerm[mb_X.size()]));
		} else {
			return P_X_given_parents;
		}
		
		
	}
		
	private ListTerm getProbs ( 
			Term X, 
			Map<Term, Term> x, 
			ListTerm parents_X, 
			Map<Term,Term> P ) {
		if ( parents_X == null ) {
			return (ListTerm) P.get(EmptyList);
		}
		// Avoiding this would be nice
		LockableList parent_values = new LockableList();
		for ( int i = 0 ; i < parents_X.size() ; i++ ) {
			parent_values.add( x.get(parents_X.get(i)) );
		}		
		ListTerm parents_assignment = Term.list(parent_values);
				
		return (ListTerm) P.get(parents_assignment);
	}
	
	private final static ListTerm EmptyList = Term.list();
	
	private NumericalTerm getProb( 
			Term X, 
			Map<Term, Term> x,  
			ListTerm values_X, 
			ListTerm parents_X,
			Map<Term,Term> P ) {
		Term x_i = x.get(X);
		
		int val_idx = values_X.asList().indexOf(x_i);
		
		if ( parents_X == null ) {
			return (NumericalTerm)P.get(EmptyList).get(val_idx);
		} else {
			// Avoiding this would be nice
			LockableList parent_values = new LockableList();
			for ( int i = 0 ; i < parents_X.size() ; i++ ) {
				parent_values.add( x.get(parents_X.get(i)) );
			}
			
			ListTerm parents_assignment = Term.list(parent_values);
		
			ListTerm P_vector = (ListTerm) P.get(parents_assignment);
			return (NumericalTerm) P_vector.get(val_idx);
		}		
	}
}
