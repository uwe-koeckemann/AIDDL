package org.aiddl.core.java.tools;

import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.representation.KeyValueTerm;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.tools.tabbing.Tabbing;

/** 
 * A simple logger.
 * 
 * @author Uwe Koeckemann
 *
 */
public class Logger {
	
	private static Map<String,PrintStream> streamMap = new HashMap<>();
	private static List<PrintStream> streams = new ArrayList<>();
	private static Map<PrintStream, Tabbing> tabbers = new HashMap<>();	
	private static int depth = 0;
	
	public static Tabbing DefaultTabbing = (int n)->{
		String r = "";
		for ( int i = 0 ; i < n ; i++ ) {
			r += "|    ";
		}
		r += " ";
		return r;
	}; 
	
	public static Tabbing OrgTabbing = (int n)->{
		String r = "*";
		for ( int i = 0 ; i < n ; i++ ) {
//			r += "|    ";
			r += "*";
		}
		r += " ";
		return r;
	}; 
	
	
	static Tabbing SimpleTabbing = (int n)->{
		String r = "";
		for ( int i = 0 ; i < n ; i++ ) {
			r += "  ";
		}
		return r;
	};

	/** Set print stream of logger.
	 * @param out
	 */
	public static void addPrintStream( PrintStream out ) {
		if ( !streams.contains(out)) {
			streams.add(out);
		}
	}
	
	/** Set print stream of logger.
	 * @param out
	 */
	public static void addPrintStream( PrintStream out, Tabbing tabFunction ) {
		if ( !streams.contains(out)) {
			streams.add(out);
			tabbers.put(out, tabFunction);
		}
	}

	/**
	 * Increase indentation of logger (for readability)
	 */
	public static void incDepth() {
		depth++;
	}
	/**
	 * Decrease indentation of logger (for readability)
	 */
	public static void decDepth() {
		depth--;
		if ( depth < 0 ) {
			throw new IllegalAccessError("Depth commands are leaking (reached negative depth for Logger messages).");
		}
	}
	/**
	 * Decrease indentation of logger (for readability)
	 */
	public static int depth() {
		return depth;
	}
		
	/** Add message to logger.
	 * @param src source of the message
	 * @param msg the message
	 */
	public static void msg( String src, String msg ) {
		if ( streams.isEmpty() ) {
			addPrintStream(System.out);
		}
		for ( int i = 0 ; i < streams.size() ; i++ ) {
			PrintStream stream = streams.get(i);
			Tabbing f_tab = tabbers.getOrDefault(stream, DefaultTabbing);
			String indentation = f_tab.tab(depth);
			msg = msg.replaceAll("\n", String.format("\n%s[%s] ",indentation, src));		
			stream.println(String.format("%s[%s] %s", indentation, src, msg));
		}
	}
	
	/** Add message to logger.
	 * @param src source of the message
	 * @param msg the message
	 */
	public static void msg( String src, String header, String content ) {
		if ( streams.isEmpty() ) {
			addPrintStream(System.out);
		}
		for ( int i = 0 ; i < streams.size() ; i++ ) {
			PrintStream stream = streams.get(i);
			Tabbing f_tab = tabbers.getOrDefault(stream, DefaultTabbing);
			String indentation = f_tab.tab(depth);
			header = header.replaceAll("\n", String.format("\n%s[%s] ",indentation, src));		
			stream.print(String.format("%s[%s] %s", indentation, src, header.replace("\n", " ")));
			stream.print("\n");
			stream.print(content);
			stream.print("\n");
		}
	}
	
	public static void startLoggingToFile( String name, Tabbing f_tab ) {
		try {
			PrintStream s = new PrintStream(name, "UTF-8");
			Logger.addPrintStream(s, f_tab);
			Logger.streamMap.put(name, s);
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static void stopLoggingToFile( String name ) {
		PrintStream s = Logger.streamMap.get(name);
		Logger.streams.remove(s);
		Logger.tabbers.remove(s);
		Logger.streamMap.remove(name);
		s.close();
	}
	
	/**
	 * Tries to print terms to be easier on the eye
	 * @param t term to be printed
	 * @param depth 
	 * @return String representation of term t
	 */
	public static String prettyPrint( Term t, int depth ) {
		String base = t.toString();
		if ( base.length() > 80 ) {
			StringBuilder sB = new StringBuilder();
			if ( t instanceof SetTerm ) {
				sB.append(SimpleTabbing.tab(depth));
				sB.append("{\n");
				
				for ( Term e : t.asCollection() ) {
					sB.append(prettyPrint(e, depth+1));
					sB.append("\n");
				}
				sB.append(SimpleTabbing.tab(depth));
				sB.append("}\n");
			}
			else if ( t instanceof ListTerm ) {
				sB.append(SimpleTabbing.tab(depth));
				sB.append("[\n");
				
				for ( Term e : t.asCollection() ) {
					sB.append(prettyPrint(e, depth+1));
					sB.append("\n");
				}
				sB.append(SimpleTabbing.tab(depth));
				sB.append("]\n");
			}
			else if ( t instanceof TupleTerm ) {
				sB.append(SimpleTabbing.tab(depth));
				sB.append("(\n");
				
				for ( int i = 0 ; i < t.size() ; i++ ) {
					sB.append(prettyPrint(t.get(i), depth+1));
					sB.append("\n");
				}
				sB.append(SimpleTabbing.tab(depth));
				sB.append(")\n");
			} else if ( t instanceof KeyValueTerm ) {
				
				sB.append(prettyPrint(t.getKey(), depth));
//				sB.append(SimpleTabbing.tab(depth));
				sB.append(":\n");
				sB.append(prettyPrint(t.getValue(), depth+1));				
		    } else {
				sB.append(SimpleTabbing.tab(depth));
				sB.append(base);
			}
			return sB.toString();
		}
		return SimpleTabbing.tab(depth) + base;
	}
}
