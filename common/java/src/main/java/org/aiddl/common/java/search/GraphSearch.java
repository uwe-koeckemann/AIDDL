package org.aiddl.common.java.search;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Stack;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.tools.StopWatch;
import org.aiddl.core.java.function.FunctionRegistry;

public class GraphSearch implements Function, InitializableFunction, ConfigurableFunction {

	PriorityQueue<CompareItem> open_list = new PriorityQueue<>();
	HashSet<Term> closed_list = new HashSet<>();
	HashSet<Term> seen = new HashSet<>();	
	Map<Term,Term> predecessors = new HashMap<Term, Term>();
	Map<Term,Integer> distance = new HashMap<>();
	Map<Term,Term> edges = new HashMap<Term, Term>();

	Stack<ListTerm> search_space = new Stack<>();  
	Stack<Integer> search_idx = new Stack<>();  
	
	int n_added = 0;
	int n_opened = 0;
	int n_pruned = 0;
	
	public static Term IsGoal = Term.sym("is-goal");
	public static Term Node = Term.sym("node");
	
	public static Term IsClosed = Term.sym("is-closed");
	public static Term Search = Term.sym("search");
	public static Term Expand = Term.sym("expand");
	public static Term Prune = Term.sym("prune");	
	public static Term Next = Term.sym("next");
	public static Term Get = Term.sym("get");	
	public static Term Distance = Term.sym("distance");	
	public static Term Path = Term.sym("path");	
	
	static Term SizeOpen = Term.sym("size-open");
	static Term SizeClosed = Term.sym("size-closed");
	static Term SizeSeen = Term.sym("size-seen");	
	static Term NumOpened = Term.sym("n-opened");
	static Term NumPruned = Term.sym("n-pruned");
	static Term NumAdded = Term.sym("n-added");
	
	private NumericalTerm omega;
	private Function compute_h;
	private Function expand;
	private Function goalTest;
	
	private List<Function> prune_functions = new LinkedList<>();
	
	private boolean includePathLength = false;
	
	private String loggerName = "GraphSearch";
	private int verbose = 0;
	
	public void setHeuristic( Function f ) {
		this.compute_h = f;
	}
	public void setExpansion( Function f ) {
		this.expand = f;
	}
	public void setGoalTest( Function f ) {
		this.goalTest = f;
	}
	public void setSearchParameters( NumericalTerm omega, boolean includePathLength ) {
		this.omega = omega;
		this.includePathLength = includePathLength;
	}
	
	@Override
	public void initialize(Term args) {
		ListTerm expansion = args.asList();
		for ( Term node : expansion ) {
			Term h = this.compute_h.apply(node);
			distance.put(node, 0);
			this.open_list.add(new CompareItem(h, node)); 
			this.seen.add(node);
		}
	}
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.loggerName = settings.getOrDefault(Term.sym("log-name"), Term.string(this.loggerName)).toString();
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		
		this.includePathLength = settings.getOrDefault(Term.sym("include-path-length"), Term.bool(false)).getBooleanValue();
		this.omega = settings.getOrDefault(Term.sym("omega"), Term.rational(1,2)).asNum();
		
		if ( this.omega.lessThan(Term.integer(0)) || this.omega.greaterThan(Term.integer(1))) {
			throw new IllegalArgumentException("Parameter omega must be between 0 and 1 (inclusive). Current value: " + this.omega);
		}
		
		this.compute_h = fReg.getFunction(settings.get(Term.sym("heuristic")));
//		if ( this.compute_h == null ) {
//			throw new IllegalArgumentException("Missing required argument: heuristic"); 
//		}
		
		this.expand = fReg.getFunction(settings.get(Term.sym("expand")));
//		if ( this.expand == null ) {
//			throw new IllegalArgumentException("Missing required argument: expand"); 
//		}
		
		this.goalTest = fReg.getFunction(settings.get(Term.sym("goal-test")));
//		if ( this.goalTest == null ) {
//			throw new IllegalArgumentException("Missing required argument: goal-test"); 
//		}
		
		if ( settings.containsKey(Prune) ) {
			for ( Term fun_ref : settings.get(Prune).asCollection() ) {
				prune_functions.add(fun_ref.asFunRef().getFunction());
			}
		}
	}	
					
	@Override
	public Term apply(Term args) {

		Term operator = args.get(0);
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(Search) ) { 
			boolean done = false;
			Term current_node = null;
			while ( !done ) {
				Term r_next = this.apply(Term.tuple(Next));
				if ( r_next.get(Node).equals(CommonTerm.NIL) ) {
					r = CommonTerm.NIL;
					done = true;
				} else if ( r_next.get(IsGoal).getBooleanValue() ) {
					done = true;
					current_node = r_next.get(Node);
					r = current_node;
				} else {
					current_node = r_next.get(Node);
					r = current_node;
				}
				if ( !done ) {
					this.apply(Term.tuple(Expand, current_node));
				}
			}
		} else if ( operator.equals(Expand) ) {
			Term source = args.get(1);
			closed_list.add(source);
			ListTerm expansion = expand.apply(source).asList(); //args.get(2).asList();
			
//			System.out.println("Expansion size: " + expansion.size());
			
			n_opened += expansion.size();
			for ( Term e : expansion ) {
				Term edge = e.get(0);
				Term destination = e.get(1);		
				NumericalTerm f;
			
				if ( !seen.contains(destination) ) {
					seen.add(destination);
					boolean is_pruned = false;
					for ( Function f_prune : prune_functions ) {
						boolean prune = f_prune.apply(destination).getBooleanValue();
						if ( prune ) {
							is_pruned =  true;
							n_pruned++;
							break;
						}
					}
					
					if ( !is_pruned ) {
						n_added++;
						
						predecessors.put(destination, source);
						edges.put(destination, edge);
						distance.put(destination, distance.get(source)+1);
						
						if ( verbose >= 1 ) {
							Logger.msg(loggerName, "Adding:\n\tSource:" + source + "\n\tEdge: " + edge + "\n\tDest:" + destination );
						}
						
						if ( includePathLength ) {
							StopWatch.start("f(n)");
							NumericalTerm h = compute_h.apply(destination).asNum(); // w*h
							NumericalTerm g = Term.integer(distance.get(destination)); // (1-w)*g
							f = g.mult(Term.integer(1).sub(omega)).add(h.mult(omega));
							StopWatch.stop("f(n)");
							
							if ( verbose >= 3 ) {
								Logger.msg(loggerName, "f = " + f + " = " + omega + "*" + h + " + " + Term.integer(1).sub(omega) + "*" + g  );
							}
						} else {
							f = compute_h.apply(destination).asNum();
							
							if ( verbose >= 3 ) {
								Logger.msg(loggerName, "f = " + f );
							}
						}
						open_list.add(new CompareItem(f, destination));
					} else {

					}
				}
			}
			r = Term.integer(n_added);
		} else if ( operator.equals(Next) ) {
			if ( verbose >= 3 ) {
				Logger.msg(loggerName, "Open list size: " + open_list.size());
			}
			if ( open_list.size() > 0 ) {
				Term node = open_list.poll().data;
				Term is_goal = goalTest.apply(node);
				r = Term.tuple(
						Term.keyVal(Term.sym("node"), node), 
						Term.keyVal(Term.sym("is-goal"), is_goal));
			} else {
				r = Term.tuple(
						Term.keyVal(Term.sym("node"), CommonTerm.NIL), 
						Term.keyVal(Term.sym("is-goal"), Term.bool(false)));
			}
		} else if ( operator.equals(IsClosed) ) {
			Term node = args.get(1);
			r =  Term.bool(closed_list.contains(node));			
		} else if ( operator.equals(Get) ) {
			Term feature = args.get(1);
			
			if ( feature.equals(NumAdded) ) {
				r = Term.integer(n_added);
			} else if ( feature.equals(NumOpened) ) {
				r = Term.integer(n_opened);
			} else if ( feature.equals(NumPruned) ) {
				r = Term.integer(n_pruned);
			} else if ( feature.equals(SizeOpen) ) {
				r = Term.integer(open_list.size());
			} else if ( feature.equals(SizeClosed) ) {
				r = Term.integer(closed_list.size());
			} else if ( feature.equals(SizeSeen) ) {
				r = Term.integer(seen.size());
			} else if ( feature.equals(Distance) ) {
				Term node = args.get(2);
				r = Term.integer(distance.get(node));
			} else if ( feature.equals(Path) ) {
				Term node = args.get(2);
				Stack<Term> s = new Stack<Term>();
				while ( node != null ) {
					if ( edges.get(node) != null ) {
						s.push(edges.get(node));
					}
					node = predecessors.get(node);
				}			
				LockableList path = new LockableList();
				while ( !s.isEmpty() ) {
					path.add(s.pop());
				}
				r = Term.list(path);						
			}
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
