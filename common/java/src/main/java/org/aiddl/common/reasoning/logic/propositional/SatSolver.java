package org.aiddl.common.reasoning.logic.propositional;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.IntegerTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.common.reasoning.logic.LogicTerm;
import org.aiddl.core.function.FunctionRegistry;

public class SatSolver implements ConfigurableFunction {
	
	String name = SatSolver.class.getSimpleName();
	boolean verbose = false;
	
	public Map<Term,Map<Term,Term>> automata = new HashMap<>();

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		SetTerm KB = (SetTerm) args.get(LogicTerm.KB);		
		SetTerm CNF = (SetTerm) args.get(LogicTerm.CNF);
	
		Map<Integer, Term> lit_map = new HashMap<>();
		List<List<Integer>> dimacs = CnfToDimacsConverter.cnf2dimacs(CNF,lit_map);
		
		List<Integer> solution = dpll(new ArrayList<Integer>(), dimacs);
		List<Term> assertedLiterals = new ArrayList<>();
		List<Term> assignment = new ArrayList<>();
		Term is_sat;
		if ( solution != null ) {
			is_sat = Term.bool(true);
//			if ( verbose ) Logger.msg(name, "Satisfiable!");
			
			for ( Integer lit : solution ) {
				if ( lit > 0 ) {
					assignment.add(Term.keyVal(lit_map.get(lit), Term.sym("true")));
					assertedLiterals.add(lit_map.get(lit));
//					if ( verbose ) Logger.msg(name, lit_map.get(lit).toString());
				} else {
					assignment.add(Term.keyVal(lit_map.get(lit), Term.sym("false")));
				}
			}
		} else {
			is_sat = Term.bool(false);
		}

		Set<Term> KB_new = new LinkedHashSet<>();
		KB.addAllTo(KB_new);
//		KB_new.addAll(KB.getCollection().getCollectionCopy());
		KB_new.addAll(assertedLiterals);
		
		List<Term> rList = new ArrayList<Term>();
		rList.add(Term.keyVal(LogicTerm.KB, Term.set(KB_new)));
		rList.add(Term.keyVal(LogicTerm.SAT, is_sat));
		rList.add(Term.keyVal(LogicTerm.Assignment, Term.list(assignment)));
		return Term.list(rList);
	}
	
	private static List<Integer> EMPTY_LIST = new ArrayList<>();
	private List<Integer> dpll ( List<Integer> a, List<List<Integer>> Phi ) {
		if ( Phi.isEmpty() ) {
			return a;
		}
		if ( Phi.contains(EMPTY_LIST) ) {
			return null;
		}
		Integer unit_clause_lit = null;
		while ( (unit_clause_lit = find_unit_clause( Phi )) != null ) {
			Phi = unit_propagate(unit_clause_lit, Phi);
			a.add(unit_clause_lit);
		}
		Integer pure_lit = null;
		while ( (pure_lit = find_pure_lit( Phi )) != null ) {
			Phi = unit_propagate(pure_lit, Phi);
			a.add(pure_lit);
		}
		if ( Phi.isEmpty() ) {
			return a;
		} 
		if ( Phi.get(0).isEmpty() ) {
			return null;
		}
		System.out.println(Phi);
		Integer lit = -Math.abs(Phi.get(0).get(0));
		
		List<List<Integer>> Phi_1 = unit_propagate(lit, Phi);
		List<Integer> a_1 = new ArrayList<>();
		a_1.addAll(a);
		a_1.add(lit);
		List<Integer> a_1_sol = dpll(a_1, Phi_1);
		if ( a_1_sol != null ) {
			return a_1_sol;
		}
		List<List<Integer>> Phi_2 = unit_propagate(-lit, Phi);
		List<Integer> a_2 = new ArrayList<>();
		a_2.addAll(a);
		a_2.add(-lit);
		return dpll(a_2, Phi_2);
	}
	
	private Integer find_pure_lit( List<List<Integer>> Phi ) {
		List<Integer> unpure = new ArrayList<>();
		List<Integer> candidates = new ArrayList<>();
		
		for ( List<Integer> clause : Phi ) {
			for ( Integer lit : clause ) {
				if ( !unpure.contains(lit) && !unpure.contains(-lit) ) {
					if ( candidates.contains(-lit) ) {
						((Collection<Integer>)candidates).remove(-lit);
						unpure.add(lit);
						unpure.add(-lit);
					} else if ( !candidates.contains(lit) ) {
						candidates.add(lit);
					}
				}
			}
		}
		if ( candidates.isEmpty() ) {
			return null;
		}
		return candidates.get(0);
	}
	
	private Integer find_unit_clause( List<List<Integer>> Phi ) {
		for ( List<Integer> clause : Phi ) {
			if ( clause.size() == 1 ) {
				return clause.get(0);
			}
		}
		return null;
	}
	
	private List<List<Integer>> unit_propagate ( Integer l, List<List<Integer>> Phi ) {
		List<List<Integer>> propagated = new ArrayList<>();
		for ( List<Integer> clause : Phi ) {
			if ( clause.contains(l) ) {
				continue;
			} 
			List<Integer> new_clause = new ArrayList<>();
			for ( Integer c_lit : clause ) {
				if ( c_lit != -l ) {
					new_clause.add(c_lit);
				}
			}
			propagated.add(new_clause);
		}
		return propagated;
	}
	
	static final Term NOT_1 = Term.sym("not");
	static final Term NOT_2 = Term.sym("!");
	
	public static List<List<Integer>> cnf2dimacs( SetTerm CNF ) {
		StringBuilder sB = new StringBuilder();
		Map<Term, Integer> varMap = new HashMap<>();
		
		int next_free_var = 1;
		
		List<List<Integer>> dimacs = new ArrayList<>();
		
		for ( Term c : CNF ) {
			System.out.println(c);
			List<Integer> dimacs_clause = new ArrayList<>();
			for ( Term l : (SetTerm)c ) {

				TupleTerm mod_and_var = getModAndVariable(l);
				IntegerTerm mod = (IntegerTerm) mod_and_var.get(0);
				Term var = mod_and_var.get(1);
				
				Integer var_id = varMap.get(var);
				if ( var_id == null ) {
					var_id = next_free_var++;
					varMap.put(var, var_id);
				}
				dimacs_clause.add(mod.getIntValue()*var_id);
				sB.append(mod.getIntValue()*var_id);
				sB.append(" ");
			}
			dimacs.add(dimacs_clause);
			sB.append("\n");
		}
		
		return dimacs;
	}
	
	public static TupleTerm getModAndVariable( Term lit ) {
		if ( !(lit instanceof TupleTerm)) {
			return Term.tuple(Term.integer(+1), lit);
		} 
		Term op = lit.get(0);
		if ( op.equals(NOT_1) || op.equals(NOT_2) ) {
			return Term.tuple(Term.integer(-1),lit.get(1));
		}
		return Term.tuple(Term.integer(+1), lit);
	}
}
