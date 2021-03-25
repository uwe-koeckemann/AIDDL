package org.aiddl.common.reasoning.resource;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

/**
 * Load scheduling problem from a .sch file. 
 * @author Uwe Köckemann
 *
 */
public class SchFileLoader implements Function {

	@Override
	public Term apply(Term args) {
		String fname = args.getStringValue();
		
		try {
			FileInputStream fstream = new FileInputStream(fname);
	
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			int i = 1;
			
			Integer num_activities = null;
			Integer num_resources = null;
			
			LockableSet constraints = new LockableSet();
			LockableSet capacities  = new LockableSet();
			LockableSet usages = new LockableSet();
			Map<Term,LockableSet> usageMap = new HashMap<>();
			
			while ((strLine = br.readLine()) != null)   {
				strLine = strLine.replace("\t", " ").trim();
				String[] tmp = strLine.split(" ");
				if ( i == 1 ) {
					// Num activities and resources
					num_activities = Integer.valueOf(tmp[0]);
					num_resources  = Integer.valueOf(tmp[1]);
				} else if ( i > 0 && i <= 1+num_activities+2 ) {
					// Precedence constraints
					Term from = Term.sym("a"+tmp[0]);
					Integer count = Integer.valueOf(tmp[2]);
					
					for ( int j = 3 ; j < 3+count ; j++ ) {
						Term to = Term.sym("a"+tmp[j]);
						Term delta = Term.integer(Integer.valueOf(tmp[j+count].substring(1).replace("]","")));
						constraints.add(Term.tuple( Term.sym("st-st"), from, to, Term.tuple(delta, Term.infPos())) ); 
					}
				} else if ( i > 1+num_activities+2 && i <= 1+2*(num_activities+2) ) {
					// Durations and usages
					Term duration = Term.integer(Integer.valueOf(tmp[2]));
					constraints.add(Term.tuple(Term.sym("duration"), Term.sym("a"+tmp[0]), Term.tuple(duration, duration)));
					for ( int j = 1; j <= num_resources; j++ ) {
						Integer usage = Integer.valueOf(tmp[2+j]);
						Term resource = Term.sym("r"+j);
						Term activity = Term.sym("a"+tmp[0]);
						if ( usage > 0 ) {
							usageMap.putIfAbsent(resource, new LockableSet());
							usageMap.get(resource).add(Term.keyVal(activity, Term.integer(usage)));
						}
					}
				} else {
					// Capacities
					for ( int j = 1 ; j <= num_resources ; j++ ) {
						Term cap = Term.integer(Integer.valueOf(tmp[j-1]));
						Term resource = Term.sym("r"+j);
						capacities.add(
								Term.keyVal(resource, 
										Term.set(
												Term.keyVal(Term.sym("min"), Term.integer(0)), 
												Term.keyVal(Term.sym("max"), cap))));
						usages.add(Term.keyVal(resource, Term.set(usageMap.get(resource))));
					}
				}
				i++;
			}
			br.close();
			Term rcpsp = Term.tuple(Term.keyVal(Term.sym("capacity"), Term.set(capacities)), 
							  Term.keyVal(Term.sym("usage"), Term.set(usages)), 
							  Term.keyVal(Term.sym("constraints"), Term.set(constraints)));
			return rcpsp;
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}
		
		return null;
	}
	

}
