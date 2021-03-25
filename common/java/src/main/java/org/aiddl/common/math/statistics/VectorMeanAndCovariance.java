package org.aiddl.common.math.statistics;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.common.math.linear_algebra.MatrixAddition;
import org.aiddl.common.math.linear_algebra.MatrixMultiplication;
import org.aiddl.common.math.linear_algebra.MatrixScalarMultiplication;
import org.aiddl.common.math.linear_algebra.MatrixTranspose;
import org.aiddl.common.math.linear_algebra.VectorSubtraction;
import org.aiddl.common.math.linear_algebra.VectorToMatrixConverter;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class VectorMeanAndCovariance implements Function {

	@Override
	public Term apply(Term args) {
		LockableList sum_vec = new LockableList();
		List<List<NumericalTerm>> COV_sum = new ArrayList<>();
		
		for ( int i = 0 ; i < args.get(0).size() ; i++ ) {
			sum_vec.add(Term.integer(0));
			
			COV_sum.add(new ArrayList<>());
			for ( int j = 0 ; j < args.get(0).size() ; j++ ) {
				COV_sum.get(i).add(Term.integer(0));
			}
		}
		
		for ( int i = 0 ; i < args.size() ; i++ ) {
			for ( int j = 0 ; j < args.get(0).size() ; j++ ) {
				sum_vec.set(j, sum_vec.get(j).asNum().add(args.get(i).get(j).asNum()));
			}
		}
		
		LockableList meanList = new LockableList();
		
		for ( int i = 0; i < sum_vec.size(); i++ ) {
			meanList.add(sum_vec.get(i).asNum().div(Term.integer(args.size())));
		}
		
		Term mean = Term.tuple(meanList);
		
		VectorToMatrixConverter v2m = new VectorToMatrixConverter();
		MatrixMultiplication mult = new MatrixMultiplication();
		MatrixAddition add = new MatrixAddition();
		MatrixScalarMultiplication msMult = new MatrixScalarMultiplication();
		
		VectorSubtraction vSub = new VectorSubtraction();
		MatrixTranspose trans = new MatrixTranspose();
		
		Term M_sum = null;
		
		for ( int i = 0 ; i < args.size() ; i++ ) {
			Term X = v2m.apply(vSub.apply(Term.tuple(args.get(i), mean)));
			Term X_t = trans.apply(X);
			
			Term XXt = mult.apply(Term.tuple(X, X_t));
			
			if ( M_sum == null ) {
				M_sum = XXt;
			} else {
				M_sum = add.apply(Term.tuple(M_sum, XXt));
			}
		}
		
		Term scalar = Term.real(1.0/((double)args.size()-1));
		Term S = msMult.apply(Term.tuple(M_sum, scalar));

		return Term.tuple(mean, S);
	}
}
