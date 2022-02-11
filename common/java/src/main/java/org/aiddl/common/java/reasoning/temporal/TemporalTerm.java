package org.aiddl.common.java.reasoning.temporal;

import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class TemporalTerm {
	
	public static Term Release = Term.sym("release");
	public static Term Deadline = Term.sym("deadline");
	public static Term Duration = Term.sym("duration");
	public static Term At = Term.sym("at");
	
	public static Term Before = Term.sym("before");
	public static Term After = Term.sym("after");
	public static Term Equals = Term.sym("equals");
	
	public static Term Meets = Term.sym("meets");
	public static Term MetBy = Term.sym("met-by");
	public static Term Starts = Term.sym("starts");
	public static Term StartedBy = Term.sym("started-by");
	public static Term Finishes = Term.sym("finishes");
	public static Term FinishedBy = Term.sym("finished-by");
	public static Term During = Term.sym("during");
	public static Term Contains = Term.sym("contains");
	public static Term Overlaps = Term.sym("overlaps");
	public static Term OverlappedBy = Term.sym("overlapped-by");
		
	public static Term StSt = Term.sym("st-st");
	public static Term StEt = Term.sym("st-et");
	public static Term EtSt = Term.sym("et-st");
	public static Term EtEt = Term.sym("et-et");
	
	public static Term ST = Term.sym("ST");
	public static Term ET = Term.sym("ET");
	
	
	public static NumericalTerm EST( Term interval, CollectionTerm intervalDomains ) {
		return intervalDomains.get(Term.tuple(ST, interval)).get(0).asNum();
	}
	
	public static NumericalTerm LST( Term interval, CollectionTerm intervalDomains ) {
		return intervalDomains.get(Term.tuple(ST, interval)).get(1).asNum();
	}
	
	public static NumericalTerm EET( Term interval, CollectionTerm intervalDomains ) {
		return intervalDomains.get(Term.tuple(ET, interval)).get(0).asNum();
	}
	
	public static NumericalTerm LET( Term interval, CollectionTerm intervalDomains ) {
		return intervalDomains.get(Term.tuple(ET, interval)).get(1).asNum();
	}
	
	public static boolean hasIntersection( Term a, Term b, CollectionTerm intervalDomains ) {
		NumericalTerm min = EST(a, intervalDomains).max(EST(b, intervalDomains));
		NumericalTerm max = EET(a, intervalDomains).min(EET(b, intervalDomains));
					
		return min.lessThan(max);
	}
}
