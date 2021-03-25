package org.aiddl.common.reasoning.logic.propositional;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.ComboIterator;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;

public class KnowledgeBase2Cnf implements ConfigurableFunction {
	
	static final Term AND_1 = Term.sym("and");
	static final Term AND_2 = Term.sym("&");
	static final Term OR_1 = Term.sym("or");
	static final Term OR_2 = Term.sym("|");
	static final Term XOR = Term.sym("xor");
	static final Term IMPL = Term.sym("=>");
	static final Term IF = Term.sym("<=");
	static final Term EQUIV = Term.sym("<=>");
	static final Term NOT_1 = Term.sym("not");
	static final Term NOT_2 = Term.sym("!");
	
	static final Set<Term> operators = new HashSet<Term>();
	
	
	boolean verbose = false;
	
	public KnowledgeBase2Cnf() {
		if ( operators.isEmpty() ) {
			operators.add(AND_1);
			operators.add(AND_2);
			operators.add(OR_1);
			operators.add(OR_2);
			operators.add(NOT_1);
			operators.add(NOT_2);
			operators.add(IMPL);
			operators.add(IF);
			operators.add(EQUIV);
			operators.add(XOR);
		}
	};
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}
	
	@Override
	public Term apply( Term args ) {
		SetTerm KB = args.asSet();
		
		List<Term> and_form = new LinkedList<>();
		and_form.add(AND_1);
//		System.out.println("INPUT");
		for ( Term t : KB ) {
			and_form.add(t);
//			System.out.println("  " + t);
		}
		Set<Term> CNF = convert(Term.tuple(and_form));
		Set<Term> outCNF = new LinkedHashSet<>();
//		System.out.println("CNF");
		for ( Term t : CNF ) {
			Term o = (t instanceof SetTerm ? t : Term.set(t));
//			System.out.println("  " + o);
			outCNF.add(o);
		}
		return Term.set(outCNF);
	}
	
	public static Set<Term> convert( Term formula ) {
		Set<Term> S = new LinkedHashSet<>();
		if ( ! (formula instanceof TupleTerm )) {
			S.add(formula);
			return S;
		}
		
		Term op = formula.get(0);
		
		if ( op.equals(AND_1) || op.equals(AND_2) ) {
			for ( int i = 1 ; i < formula.size() ; i++ ) {
				S.addAll(convert(formula.get(i)));
			}
		} else if ( op.equals(OR_1) || op.equals(OR_2) ) {
			List<List<Term>> combo_in = new ArrayList<>();
			for ( int i = 1 ; i < formula.size() ; i++ ) {
				List<Term> P_i = new LinkedList<>();
				P_i.addAll(convert(formula.get(i)));
				combo_in.add(P_i);
			}
			ComboIterator<Term> combos = new ComboIterator<>(combo_in);
			LockableSet or_form;
			for ( List<Term> combo : combos ) {
				or_form = new LockableSet();
//				or_form.add(OR_1);
				for ( Term t : combo ) {
					if ( isFormula(t) ) {
						Term sub_op = t.get(0);
						if ( sub_op.equals(OR_1) || sub_op.equals(OR_2) ) {
							for ( int i = 1 ; i < t.size() ; i++ ) {
								or_form.add(t.get(i));
							}
						} else {
							or_form.add(t);
						}
					} else {
						if ( t instanceof SetTerm ) {
							or_form.addAll(t.asSet().getCollectionCopy());
						} else {
							or_form.add(t);
						}
					}
				}
				S.add(Term.set(or_form));
			}
		} else if ( op.equals(XOR) ) {
			List<Term> form = new ArrayList<>();
			form.add(OR_1);
			for ( int i = 1 ; i < formula.size() ; i++ ) {
				LockableList sub_form = new LockableList();
				sub_form.add(AND_1);
				for ( int j = 1 ; j < formula.size() ; j++ ) {
					if ( i == j ) {
						sub_form.add(formula.get(j));
					} else {
						sub_form.add(Term.tuple(NOT_1, formula.get(j)));
					}
				}
				form.add(Term.tuple(sub_form));
			}
			S = convert(Term.tuple(form));
		} else if ( op.equals(IMPL) ) {
			S = convert(
					Term.tuple(OR_1, 
							Term.tuple(NOT_1, formula.get(1)), 
							formula.get(2)));
		} else if ( op.equals(IF) ) {
			S = convert(
					Term.tuple(OR_1, 
							formula.get(1),
							Term.tuple(NOT_1, formula.get(2))));
			
		} else if ( op.equals(EQUIV) ) {
			S = convert(
					Term.tuple(OR_1, 
							Term.tuple(AND_1, formula.get(1), formula.get(2)),
							Term.tuple(AND_1, Term.tuple(NOT_1, formula.get(1)), Term.tuple(NOT_1, formula.get(2)))));
		} else if ( op.equals(NOT_1) || op.equals(NOT_2) ) {
			Term arg = formula.get(1);
			
			if ( !isFormula(arg) ) {
				S.add(formula);
			} else {
				Term sub_op = arg.get(0);
				if ( sub_op.equals(OR_1) || sub_op.equals(OR_2)) {
					List<Term> and_form = new ArrayList<>();
					and_form.add(AND_1);
					for ( int i = 1 ; i < arg.size() ; i++ ) {
						and_form.add(Term.tuple(NOT_1, arg.get(i)));
					}
					S = convert(Term.tuple(and_form));
				} else if ( sub_op.equals(AND_1) || sub_op.equals(AND_2)) {
					List<Term> or_form = new ArrayList<>();
					or_form.add(OR_1);
					for ( int i = 1 ; i < arg.size() ; i++ ) {
						or_form.add(Term.tuple(NOT_1, arg.get(i)));
					}
					S = convert(Term.tuple(or_form));
				} else if ( sub_op.equals(NOT_1) || sub_op.equals(NOT_2)) {
					S =  convert(arg.get(1));
				} else {
					Set<Term> conv = convert(arg.get(1));
					List<Term> or_form = new ArrayList<>();
					or_form.add(OR_1);
					for ( Term s : conv ) {
						or_form.add(Term.tuple(NOT_1, s));
					}
					S = convert(Term.tuple(or_form));
				}
			}
		} else {
			S.add(formula);
			
		}
		return S;
	}
	
	public static  boolean isFormula( Term t ) {
		if ( !(t instanceof TupleTerm)) {
			return false;
		} 
		Term op = t.get(0);
		if ( !operators.contains(op) ) {
			return false;
		}
		return true;
	}
}
