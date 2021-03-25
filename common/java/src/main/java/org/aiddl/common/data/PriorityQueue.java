package org.aiddl.common.data;

import java.util.Comparator;
import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;

public class PriorityQueue implements Function, ConfigurableFunction {

	
	java.util.PriorityQueue<CompareItem> Q = new java.util.PriorityQueue<>();
	
	static Term Add = Term.sym("add");
	static Term AddAll = Term.sym("add-all");
	
	static Term Pop = Term.sym("pop");
	static Term Peek = Term.sym("peek");
	
	static Term Size = Term.sym("size");
	
	static Term f_extract_att = Term.sym("compare");
	static Term ExtractArg = Term.sym("#arg");
	
	Term f_extract;
	
	private Function eval;
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.f_extract = settings.getOrDefault(f_extract_att, ExtractArg);
		this.eval = fReg.getFunction(settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL));
	}
	
	@Override
	public Term apply(Term args) {

		Term operator = args.get(0);
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(Add) ) {
			Substitution s = new Substitution();
			Term arg = args.get(1);
			s.add(ExtractArg, arg);
			Term f = this.f_extract.substitute(s);
			Term key = eval.apply(f);
			Q.add(new CompareItem(key, arg));
		} else if ( operator.equals(AddAll) ) {
			Term arg = args.get(1);
			for ( Term e : arg.asCollection() ) {
				Term key = eval.apply(e);
				Q.add(new CompareItem(key, e));
			}
		} else if ( operator.equals(Pop) ) {
			if ( !Q.isEmpty() ) {
				r = Q.poll().data;
			}
		} else if ( operator.equals(Peek) ) {
			if ( !Q.isEmpty() ) {
				r = Q.peek().data;
			}
		} else if ( operator.equals(Size) ) {
			r = Term.integer(Q.size());
		}  
		
		return r;
	}
	
	private class CompareItem implements Comparable<CompareItem> {
		
		public Term key;
		public Term data;
		
		public CompareItem( Term key, Term data ) {
			this.key = key;
			this.data = data;
		}

		@Override
		public int compareTo(CompareItem o) {
			return Comparator.compare(this.key, o.key);
		}
		
		private Comparator<Term> Comparator = new Comparator<Term>() {
			@Override
			public int compare(Term o1, Term o2) {
				if ( o1 instanceof NumericalTerm && o2 instanceof NumericalTerm ) {
					return ((NumericalTerm)o1).compareTo(((NumericalTerm)o2));
				} else if ( o1 instanceof TupleTerm && o2 instanceof TupleTerm ) {
					if ( o1.size() == o2.size() ) {
						for ( int i = 0 ; i < o1.size() ; i++ ) {
							if ( !(o1.get(i) instanceof NumericalTerm) || !(o2.get(i) instanceof NumericalTerm) ) {
								throw new IllegalArgumentException("Cannot compare " + o1 + " to " + o2 + ".\n"
										+ "\tCan only compare numerical terms or tuples of numerical terms\n");
							}
							NumericalTerm x1 = (NumericalTerm)o1.get(i);
							NumericalTerm x2 = (NumericalTerm)o2.get(i);
							int r = x1.compareTo(x2);
							if ( r != 0 ) {
								return r;
							}
						}
						return 0;
					}
				}
				throw new IllegalArgumentException("Cannot compare " + o1 + " to " + o2 + ".\n"
						+ "\tCan only compare numerical terms or tuples of numerical terms\n");
			}
		};
		
	}
	
	
}
