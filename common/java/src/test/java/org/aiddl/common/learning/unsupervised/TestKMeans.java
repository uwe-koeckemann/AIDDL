package org.aiddl.common.learning.unsupervised;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.common.learning.linear_regression.LinearRegression;
import org.aiddl.common.learning.linear_regression.OrdinaryLeastSquaresRegression;
import org.aiddl.common.learning.testing.CrossValidation;
import org.aiddl.common.learning.testing.DataShuffler;
import org.aiddl.common.learning.unsupervised.clustering.KMeansClustering;
import org.aiddl.common.math.linear_algebra.VectorAddition;
import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.Function;

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