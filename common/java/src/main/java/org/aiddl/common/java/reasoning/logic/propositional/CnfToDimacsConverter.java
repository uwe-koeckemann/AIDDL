package org.aiddl.common.java.reasoning.logic.propositional;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.representation.IntegerTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

public class CnfToDimacsConverter {
	
	static final Term NOT_1 = Term.sym("not");
	static final Term NOT_2 = Term.sym("!");
	
	public static List<List<Integer>> cnf2dimacs( SetTerm CNF, Map<Integer, Term> lit_name_map ) {
		Map<Term, Integer> varMap = new HashMap<>();
		
		int next_free_var = 1;
		
		List<List<Integer>> dimacs = new ArrayList<>();
		
		for ( Term c : CNF ) {
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
				
				lit_name_map.putIfAbsent(mod.getIntValue()*var_id, l);
			}
			dimacs.add(dimacs_clause);
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
