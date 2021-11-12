package org.aiddl.common.java.reasoning.temporal.simple_temporal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class STPSolver implements Function, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.temporal.stp.solver");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply( Term stp ) {
		SetTerm X = (SetTerm) stp.get(0);
		SetTerm C = (SetTerm) stp.get(1);
		
		int n = X.size();
		
		NumericalTerm O = Term.integer(0);
		NumericalTerm H = Term.infPos();
		
		Map<Term,NumericalTerm> tpMap = new HashMap<>();
		Map<Long,Term> tpMapInv = new HashMap<>();
		long nextTP = 0L;
		for ( Term x : X ) {
			tpMap.put(x, Term.integer(nextTP));
			tpMapInv.put(nextTP, x);
			nextTP++;
		}
		
		List<ConstraintMap> Dmap = getDmap(n,C, tpMap);
				
		NumericalTerm[] lower;
		NumericalTerm[] upper;
		lower = new NumericalTerm[n];
		upper = new NumericalTerm[n];
		boolean[] update = new boolean[n];
		NumericalTerm[] pl = new NumericalTerm[n];
		NumericalTerm[] pu = new NumericalTerm[n];
		
		for ( int i = 0 ; i < n ; i++ ) {
			lower[i] = O;
			upper[i] = H;
			update[i] = true;
			
			pl[i] = null;
			pu[i] = null;
		}
		
		Integer v = -1;
		while ( v != null ) {
			v = null;
			
			int i = -1;
			
			while ( i+1 < n ) {
				i += 1;
				if ( update[i] ) {
					v = i;
					update[i] = false;
					
					for ( NumericalTerm[] con : Dmap.get(i).in ) {
						Result r = applyCon(lower, upper, con);
							
						if ( !r.isConsistent ) {
							return CommonTerm.NIL;
						}
						if ( r.change[0] ) {
							update[con[0].getIntValue()] = true;
						}
						if ( r.change[1] ) {
							update[con[1].getIntValue()] = true;
						}
						
						if ( r.changeDetailed[0] ) {
							pl[con[0].getIntValue()] = con[1]; 
							if ( hasCycle( pl, con[0] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[2] ) {
							pl[con[1].getIntValue()] = con[0]; 
							if ( hasCycle( pl, con[1] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[1] ) {
							pu[con[0].getIntValue()] = con[1]; 
							if ( hasCycle( pu, con[0] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[3] ) {
							pu[con[1].getIntValue()] = con[0]; 
							if ( hasCycle( pu, con[1] ) ) {
								return CommonTerm.NIL;
							}
						}
					}
					for ( NumericalTerm[] con : Dmap.get(i).out ) {
						Result r = applyCon(lower, upper, con);
						
						if ( !r.isConsistent ) {
							return CommonTerm.NIL;
						}
						if ( r.change[0] ) {
							update[con[0].getIntValue()] = true;
						}
						if ( r.change[1] ) {
							update[con[1].getIntValue()] = true;
						}
						
						if ( r.changeDetailed[0] ) {
							pl[con[0].getIntValue()] = con[1]; 
							if ( hasCycle( pl, con[0] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[2] ) {
							pl[con[1].getIntValue()] = con[0]; 
							if ( hasCycle( pl, con[1] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[1] ) {
							pu[con[0].getIntValue()] = con[1]; 
							if ( hasCycle( pu, con[0] ) ) {
								return CommonTerm.NIL;
							}
						}
						if ( r.changeDetailed[3] ) {
							pu[con[1].getIntValue()] = con[0]; 
							if ( hasCycle( pu, con[1] ) ) {
								return CommonTerm.NIL;
							}
						}
					}
				}
			}
		}
		
		List<Term> intervals = new ArrayList<>();
		for ( int i = 0 ; i < n ; i++ ) {
			intervals.add(Term.keyVal(tpMapInv.get(Long.valueOf(i)), Term.tuple(lower[i],upper[i])));
		}
		return Term.list(intervals);
	}

	private class ConstraintMap {
		public List<NumericalTerm[]> in = new ArrayList<NumericalTerm[]>();
		public List<NumericalTerm[]> out = new ArrayList<NumericalTerm[]>();
	}
	
	private List<ConstraintMap> getDmap( int n, SetTerm D, Map<Term,NumericalTerm> varMap ) {
		List<ConstraintMap> Dmap = new ArrayList<ConstraintMap>();
		for ( int i = 0 ; i < n ; i++ ) {
			Dmap.add(new ConstraintMap());
		}
		for ( Term c_tuple : D ) {
			NumericalTerm[] c = new NumericalTerm[4];
			c[0] = varMap.get(c_tuple.get(0));
			c[1] = varMap.get(c_tuple.get(1));
			c[2] = c_tuple.get(2).asNum();
			c[3] = c_tuple.get(3).asNum();
			
			Dmap.get(c[0].getIntValue()).out.add(c);
			Dmap.get(c[1].getIntValue()).in.add(c);
		}
			
		return Dmap;
	}
	
	private class Result {
		public boolean isConsistent;
		public boolean[] change = new boolean[2];
		public boolean[] changeDetailed = new boolean[4];
		
		public Result( boolean isCon, boolean[] change, boolean[] changeDetailed) {
			this.isConsistent = isCon;
			this.change = change;
			this.changeDetailed = changeDetailed;
		}
	}
	
	private Result applyCon( NumericalTerm[] lower, NumericalTerm[] upper, NumericalTerm[] con ) {
		int tp1 = con[0].getIntValue();
		int tp2 = con[1].getIntValue();
		
		NumericalTerm newTP1l = lower[tp2].sub(con[3]);
		NumericalTerm newTP1h = upper[tp2].sub(con[2]);
		NumericalTerm newTP2l = lower[tp1].add(con[2]);
		NumericalTerm newTP2h = upper[tp1].add(con[3]);
			
		boolean[] change = new boolean[2];
		change[0] = false;
		change[1] = false;
		
		boolean[] changeDetailed = new boolean[4];
		changeDetailed[0] = false;
		changeDetailed[1] = false;
		changeDetailed[2] = false;
		changeDetailed[3] = false;
		
		
		if ( newTP1l.greaterThan(lower[tp1]) ) {
			change[0] = true;
			changeDetailed[0] = true;
			lower[tp1] = newTP1l;
		}
		if ( newTP1h.lessThan(upper[tp1]) ) {
			change[0] = true;
			changeDetailed[1] = true;
			upper[tp1] = newTP1h;
		}
			
		if ( newTP2l.greaterThan(lower[tp2]) ) {
			change[1] = true;
			changeDetailed[2] = true;
			lower[tp2] = newTP2l;
		}
			
		if ( newTP2h.lessThan(upper[tp2]) ) {
			change[1] = true;
			changeDetailed[3] = true;
			upper[tp2] = newTP2h;
		}

		return new Result(lower[tp1].lessThanEq(upper[tp1]) && lower[tp2].lessThanEq(upper[tp2]), change, changeDetailed);
	}	
	
	private boolean hasCycle( NumericalTerm[] a, NumericalTerm start ) {
		List<NumericalTerm> seen = new ArrayList<NumericalTerm>(); 
		int u = start.getIntValue();
		
		while ( a[u] != null ) {
			if ( seen.contains(a[u]) ) {
				return true;
			}
			seen.add(a[u]);
			u = a[u].getIntValue();
		}
		return false;
	}
}