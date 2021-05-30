package org.aiddl.common.learning.decision_tree;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.FunctionGenerator;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class ID3 implements ConfigurableFunction, FunctionGenerator, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.decision-tree.learner");
	
	boolean includeAllLeafes = false;
	
	String name = ID3.class.getSimpleName();
	Integer verbose = 0;
	
	private Term atts = null;
	private Term DT = null;
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.includeAllLeafes = Term.sym("true").equals(settings.get(Term.sym("includeAllLeafes")));
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Function generate() {
		DecisionTreeClassifier classifier = new DecisionTreeClassifier();
		classifier.initialize(Term.tuple(DT, atts));
		return classifier;
	}

	@Override
	public Term apply( Term problem ) {
//		TupleTerm mlProblem = (TupleTerm) problem.get(Term.sym(":ml-problem"));
		
		
		ListTerm attsTerm = (ListTerm) problem.get(LearningTerm.Attributes);
		Term label = problem.get(LearningTerm.Label);
		ListTerm examples;
		
		this.atts = attsTerm;
		
		if ( problem.get(LearningTerm.Data) instanceof ListTerm ) {
			examples = (ListTerm) problem.get(LearningTerm.Data);
		} else {
			SetTerm exampleSet = (SetTerm) problem.get(LearningTerm.Data);
			List<ListTerm> setUnpack = new ArrayList<>();
			for ( Term e : exampleSet ) {
				setUnpack.add((ListTerm) e);
			}
			examples = Term.list(setUnpack); //Term.list(exampleSet.toListTerm(Term.class).getList());
			//examples = Term.list(mlProblem.get(Term.sym(":data")).toSetTerm());
		}
		int labelIdx = -1;
		
		List<Term> attNames = new ArrayList<>(attsTerm.size());
		List<Term> defNames = new ArrayList<>(attsTerm.size());
		List<Integer> attributes = new ArrayList<>();
		List<List<Term>> values = new ArrayList<>();
		for ( int i = 0 ; i < attsTerm.size() ; i++ ) {
			attNames.add(attsTerm.get(i).get(0));
			defNames.add(attsTerm.get(i).get(1));
			
			if ( attNames.get(i).equals(label) ) {
				labelIdx = i;
			} else {
				attributes.add(i);
			}
			
			values.add(new ArrayList<>());
			for ( Term dp : examples ) {
				if ( !values.get(i).contains(dp.get(i)) ) {
					values.get(i).add(dp.get(i));
				}
			}
		}
		
		if ( labelIdx == -1 ) {
			throw new IllegalArgumentException("Label " + label + " not found in list or attributes:\n" + attNames);
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "Learning " + label);
			if ( verbose >= 2 ) {
				Logger.msg(name, "Training data:");
				Logger.incDepth();
				for ( Term example : examples ) {
					Logger.msg(name, example.toString());	
				}
				Logger.decDepth();
			}
		}		
		Term decisionTree = runID3(examples, labelIdx, attributes, attNames, values);
				
		if ( verbose >= 1) {
			Logger.msg(name, decisionTree.toString());
		}
		this.DT = decisionTree;
		return decisionTree;
	}
	
	private Term runID3( ListTerm examples, int labelIdx, List<Integer> attributes, List<Term> attNames, List<List<Term>> values ) {
		Map<Term, ListTerm> partitions = this.partition(examples, labelIdx, values.get(labelIdx));
		
		Set<Term> leftovers = new LinkedHashSet<>();
		
		int max = 0;
		Term mostCommon = null;
		for ( Term k : partitions.keySet() ) {
			if ( partitions.get(k).size() > max ) {
				max = partitions.get(k).size();
				mostCommon = k;
			} 
			if ( partitions.get(k).size() > 0 ) {
				leftovers.add(k);
			} 
		}
		
		if ( attributes.isEmpty() || max == examples.size() ) {
			if ( includeAllLeafes && leftovers.size() > 1 ) {
				return Term.set(leftovers);
			}
			return mostCommon;
		}

		double max_gain = -1.0;
		int argMax = -1;
		for ( Integer attIdx : attributes ) {
			double gain = information_gain(examples, attIdx, labelIdx, values.get(attIdx));
			if ( gain > max_gain ) {
				max_gain = gain;
				argMax = attIdx;
			}
		}
//		System.out.println("Selected attribute: " + attNames.get(argMax));
		List<TupleTerm> children = new ArrayList<TupleTerm>();
		Map<Term, ListTerm> attPartitions = partition(examples, argMax, values.get(argMax));
		for ( Term v : values.get(argMax) ) {
			Term condition = Term.tuple(Term.sym("="), attNames.get(argMax), v);
			
			if ( attPartitions.get(v).size() == 0 ) {
				
				if ( includeAllLeafes && leftovers.size() > 1 ) {
					children.add( Term.tuple(condition, Term.set(leftovers)));
				} else {
					children.add( Term.tuple(condition, mostCommon));	
				}
			} else {
				List<Integer> newAtts = new ArrayList<>();
				for ( Integer idx : attributes ) {
					if ( idx != argMax ) {
						newAtts.add(idx);
					}
				}
				Term subTree = runID3( attPartitions.get(v), labelIdx, newAtts, attNames, values );
				children.add( Term.tuple(condition, subTree) );
			}
		}
		return Term.list(children);
	}
	
	private double information_gain ( ListTerm examples, int attIdx, int labelIdx, List<Term> values ) {
		Map<Term, ListTerm> partitions = this.partition(examples, attIdx, values);
		
		double num_elements = Double.valueOf(examples.size());
		double gain = entropy(examples, labelIdx);
		for ( ListTerm p : partitions.values() ) {
			double value_count = Double.valueOf(p.size());
			gain -= (value_count / num_elements) * entropy(p, labelIdx);
		}
		System.out.println("Gain for : " + attIdx + " is " + gain);
		return gain;
	}
	
	private Map<Term,ListTerm> partition ( ListTerm examples, int attIdx, List<Term> values ) {
		Map<Term, List<ListTerm>> partitions = new HashMap<>();
		for ( Term v : values ) {
			partitions.put(v, new ArrayList<>());
		}
		for ( Term sample : examples ) {
			Term value = sample.get(attIdx);
			partitions.get(value).add((ListTerm) sample);
		}
		Map<Term, ListTerm> r = new HashMap<>();
		for ( Term k : partitions.keySet() ) {
			r.put(k, Term.list(partitions.get(k)));
		}
		return r;
		
	}
	
	private double entropy( ListTerm examples, int labelIdx ) {
		Map<Term, Integer> count = new HashMap<>();
		
		for ( Term sample : examples ) {
			Term value = sample.get(labelIdx);
			if ( !count.containsKey(value) ) {
				count.put(value, 0);
			}
			count.put(value, count.get(value)+1) ;
		}
		
		double num_elements = Double.valueOf(examples.size());
		double entropy = 0.0;
		for ( Integer c : count.values() ) {
			double p_i = Double.valueOf(c) / num_elements;
			entropy += -p_i * log2(p_i);
		}
		
		return entropy;
	}
	
	private static double LOG2 = Math.log(2.0);
	
	private static double log2( double x ) {
		return Math.log(x) / LOG2;
	}
}
