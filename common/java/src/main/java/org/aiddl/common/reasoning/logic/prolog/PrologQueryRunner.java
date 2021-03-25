package org.aiddl.common.reasoning.logic.prolog;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.logic.LogicTerm;
import org.aiddl.common.tools.ExecuteSystemCommand;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.StringTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.representation.VariableTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Global;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;

public class PrologQueryRunner implements ConfigurableFunction, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.logic.prolog.query");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	String name = PrologQueryRunner.class.getSimpleName();
	Integer verbose = 0;
	
	public void setVerbose( int verbosity ) {
		this.verbose = verbosity;
	}

	@Override
	public Term apply(Term args) {
		Term KB = args.get(LogicTerm.KB);
		Term query = args.get(LogicTerm.Query);
		
		Map<String, Term> prolog2term = new HashMap<String, Term>();
		
		StringBuilder aiddl2prolog = new StringBuilder();
		/**
		 * Compose KB
		 */
		for ( Term kb_entry : KB.asSet() ) {
			if ( kb_entry instanceof StringTerm ) { // Prolog file
				String f_name = kb_entry.getStringValue(); 
				String kb_str = loadProlog(f_name);
				aiddl2prolog.append(kb_str);
			} else { // AIDDL Atoms
				String pStr = atom2prolog(kb_entry, prolog2term);
				aiddl2prolog.append(pStr + ".\n");
			}
		}
		
		String kbFile = Global.workDir() + "/kb.prolog";
		String answerFile = Global.workDir() + "/answer.prolog";
		String runFile = Global.workDir() + "/run.prolog";
		
		/**
		 * Create query
		 */
		List<Term> variables = new ArrayList<>(); 
		
		String queryPred = "queryPred(<VARS>) :- <QPREDS>.";
		String bagOf = String.format("start :- ['kb.prolog'], "
				+ "bagof( <VARS_BAG>, queryPred(<VARS>),QueryResult ) -> tell('answer.prolog'), "
				+ "write(QueryResult) ; "
				+ "tell('answer.prolog'), "
				+ "write('[-]').", kbFile, answerFile, answerFile);
		StringBuilder queryPredBuilder = new StringBuilder();
		for ( int i = 0 ; i < query.size() ; i++ ) {
			Term q_entry = query.get(i);
			getVariables(q_entry, variables);
			String pStr = atom2prolog(q_entry, prolog2term);
			queryPredBuilder.append(pStr);
			if ( i < query.size()-1 ) 
				queryPredBuilder.append(", ");
		}
		StringBuilder varBuilder = new StringBuilder();
		StringBuilder varBagBuilder = new StringBuilder();

		for ( int i = 0 ; i < variables.size() ; i++ ) {
			Term var = variables.get(i);
			String pStr = var.toString().replace("?", "Var_");
			prolog2term.put(pStr, var);
			varBuilder.append(pStr);
			varBagBuilder.append(pStr);
			if ( i < variables.size()-1 ) {
				varBuilder.append(", ");
				varBagBuilder.append("/");
			}
		}
		
		queryPred = queryPred.replace("<VARS>", varBuilder.toString());
		queryPred = queryPred.replace("<QPREDS>", queryPredBuilder.toString());
		bagOf = bagOf.replace("<VARS_BAG>", varBagBuilder.toString());
		bagOf = bagOf.replace("<VARS>", varBuilder.toString());
		
		aiddl2prolog.append(queryPred);
		
		dump2file(aiddl2prolog.toString(), kbFile); //Global.workDir() + "/kb.prolog");
		dump2file(bagOf.toString(), runFile); // Global.workDir() + "/run.prolog");
		
		if ( verbose > 0 ) Logger.msg(name, "Query: " + queryPred);
				
		/**
		 * Run query
		 */
		// swipl -s run.prolog -g start -s runFile -t halt
		// yap -L runFile
		String[] con_out = ExecuteSystemCommand.call(Global.workDir(), "swipl -s "+ runFile + " -g start -t halt");		
		/**
		 * Create output
		 */
		Collection<Substitution> r = this.getResults(variables, prolog2term); 
		
		if ( r == null ) {
			if ( verbose > 0 ) Logger.msg(name, "Query provided no results.");
			return CommonTerm.NIL;
		}
		
		LockableList rList = new LockableList();
		for ( Substitution s : r ) {
			LockableList subList = new LockableList();
			for ( Term t_from : variables ) {
				subList.add(Term.keyVal(t_from, s.substitute(t_from)));
			}
			Term e = Term.list(subList);
			if ( !rList.contains(e) ) {
				rList.add(e);
			}
		}
		ListTerm results = Term.list(rList);
		if ( verbose > 0 ) Logger.msg(name, "Results: " + results);
		
		return results;
	}
	
	private void dump2file( String s, String fname ) {
		try {
			FileWriter fstream = new FileWriter(fname);
			BufferedWriter out = new BufferedWriter(fstream);
			out.write(s);
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	private void getVariables( Term t, List<Term> L ) {
		if ( t.isVariable() ) {
			if ( !L.contains(t) ) {
				L.add(t);
			}
		} else if ( t instanceof CollectionTerm ) {
			for ( Term t_arg : t.asCollection() ) {
				getVariables(t_arg, L);
			}
		} else if ( t instanceof TupleTerm ) {
			for ( int i = 0 ; i < t.size() ; i++ ) {
				getVariables(t.get(i), L);
			}
		}
	}
	
	private String loadProlog( String fname  ) {
		FileInputStream fstream;
		StringBuilder sB = new StringBuilder();
		
		try {
			fstream = new FileInputStream(fname);
	
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			while ((strLine = br.readLine()) != null)   {
				sB.append(strLine);
				sB.append("\n");
			}
			br.close();
			return sB.toString();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}
		return null;
	}
	
	private String atom2prolog( Term a, Map<String, Term> prolog2term ) {
		StringBuilder sB = new StringBuilder();
		String name = a.get(0).toString().toLowerCase();
		
		prolog2term.put(name, a.get(0));
		
		sB.append(name);
		sB.append("(");
		String s = null;
		
		for ( int i = 1 ; i < a.size() ; i++ ) {
			Term arg = a.get(i);
			
			if ( arg instanceof SymbolicTerm ) {
				s = "'" + arg.toString() + "'";
				//s.toLowerCase();
			} else if ( arg instanceof VariableTerm ) {
				s = arg.toString();
				s = s.replace("?", "Var_");
			} else if ( arg instanceof TupleTerm ) {
				s = atom2prolog(arg, prolog2term);
			}
			prolog2term.put(s, arg);
			sB.append(s);
			if ( i < a.size()-1 )
				sB.append(", ");
		}
		sB.append(")");
		return sB.toString();
	}
	
	private Collection<Substitution> getResults( List<Term> queryVars, Map<String, Term> prolog2aiddl ) {
		ArrayList<Substitution> resultingSubs = new ArrayList<Substitution>();
		
		try {
			String answer_fname = Global.workDir() + "/answer.prolog";
			FileInputStream fstream = new FileInputStream(answer_fname);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			ArrayList<String> allSubstitutions;
			
			
			while ((strLine = br.readLine()) != null)   {
				if ( strLine.trim().equals("[queryPred]") || strLine.trim().equals("[queryPred,queryPred]") ) {
					br.close();
					return new ArrayList<Substitution>();
				} else if ( strLine.trim().equals("[-]")) {
					br.close();
					return null;
				}
				
//				System.out.println("<START>");
//				System.out.println(strLine);
//				System.out.println("<COMPLEX SPLIT>");
//				for ( String s : SimpleParsing.complexSplit(strLine.substring(1,strLine.length()-1), ",") ) {
//					System.out.println(s);
//				}

				allSubstitutions = complexSplit(strLine.substring(1,strLine.length()-1), ","); // strLine.substring(1,strLine.length()-1).split(",");
					
				for ( int i = 0 ; i < allSubstitutions.size(); i++ ) {
//					String s = allSubstitutions.get(i);
//					System.out.println(s + " -> " + Term.parse(SimpleParsing.convertTermFormat(s)));
//					s = s.replace("[", "{");
//					s = s.replace("]", "}");
//					System.out.println(s + " -> " + Term.parse(SimpleParsing.convertTermFormat(s)));
						
					
					
					String[] constants = allSubstitutions.get(i).replace("[","list(").replace("]",")").replace("/", "<####>").split("<####>");
											
					Substitution theta = new Substitution();

					for ( int k = 0 ; k < constants.length; k++ ) {	
						Term realVal = prolog2aiddl.get(constants[k]);
						
						if ( constants[k].contains("(") ) {
							constants[k] = convertTermFormat(constants[k]);
						}
						realVal = Parser.ParseTerm(constants[k]);
						
						theta.add( queryVars.get(k), realVal );
					}
					resultingSubs.add(theta);
				}
			}
			
			br.close();
			in.close();

		} catch (Exception e){
			e.printStackTrace();
			System.exit(0);
		}

		if ( resultingSubs.size() > 0 ) {
			return resultingSubs;
		} else {
			return null;
		}
	}
	
	/**
	 * Split String around comma that are not inside parenthesis.
	 * @param resultsLine
	 * @param separator 
	 * @return list of strings
	 */
	private static ArrayList<String> complexSplit( String resultsLine, String separator ) {
		ArrayList<String> results = new ArrayList<String>();
				
		String tmp = "";
		int parenthesisDepth = 0;
		
		for ( int i = 0 ; i < resultsLine.length() ; i++ ) {
			String currentChar = String.valueOf(resultsLine.charAt(i));
			if ( currentChar.equals(separator) && parenthesisDepth == 0 ) {
				results.add(tmp);
				tmp = "";
	
				continue;
			} else if ( currentChar.equals("(") ) {
				parenthesisDepth++;
			} else if ( currentChar.equals(")") ) {
				parenthesisDepth--;
			} else if ( currentChar.equals("[") ) {
				parenthesisDepth++;
			} else if ( currentChar.equals("]") ) {
				parenthesisDepth--;
			}
			tmp += currentChar;
		}
		results.add(tmp);
		
		return results;
	}
	
	
	/**
	 * Converts String representation of term in format f(x,y,g(z)) to 
	 * s-expression format: (f x y (g z))
	 * @param in Prolog style term
	 * @return String in s-expression format
	 */
	private static String convertTermFormat( String in ) {
		String tmp = " " + in.replace(",", " ");
		
		int lastSpaceAt = 0;
		
		for ( int i = 0 ; i < tmp.length(); i++ ) {
			if ( tmp.charAt(i) == ' ' ) {
				lastSpaceAt = i;
			} else if ( tmp.charAt(i) == '(' ) {
				tmp = tmp.substring(0, lastSpaceAt) + " (" + tmp.substring(lastSpaceAt+1,i) + " " + tmp.substring(i+1);  
			}
		}
		return tmp.substring(1);
	}

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.name = settings.getOrDefault(Term.sym("log-name"), Term.string(this.name)).toString();
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
}
