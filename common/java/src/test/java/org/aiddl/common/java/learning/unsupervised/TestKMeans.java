package org.aiddl.common.java.learning.unsupervised;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.aiddl.common.java.learning.testing.DataShuffler;
import org.aiddl.common.java.learning.unsupervised.clustering.KMeansClustering;
import org.aiddl.common.java.math.linear_algebra.VectorAddition;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestKMeans extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	
	public void testKMeans01() {
		Random r = new Random();
		LockableList data = new LockableList();
		
		int n_data = 1000;
		
		VectorAddition add = new VectorAddition();
		
		List<Term> centers = new ArrayList<>();
		centers.add(Term.tuple(Term.real(-1.0), Term.real(1.0)));
		centers.add(Term.tuple(Term.real(1.0), Term.real(1.0)));
		centers.add(Term.tuple(Term.real(0.0), Term.real(0.0)));
		
		double mean = 0.0;
		double std = 1.0;
		
		for ( int i = 0 ; i < n_data ; i++ ) {
			Term center = centers.get(i % 3);
			Term x = Term.tuple(Term.real(mean+std*r.nextGaussian()),
								Term.real(mean+std*r.nextGaussian()) );
			data.add(add.apply(Term.tuple(center, x)));
					
		}
		
		Term d = Term.list(data);
		
		DataShuffler shuffle = new DataShuffler();
		d = shuffle.apply(d);
		
//		System.out.println(Logger.prettyPrint(d, 0));
		
		KMeansClustering kmeans = new KMeansClustering();
		
		Term clusters = kmeans.apply(Term.tuple(Term.integer(3), d));
		
//		System.out.println(clusters);
	}
}