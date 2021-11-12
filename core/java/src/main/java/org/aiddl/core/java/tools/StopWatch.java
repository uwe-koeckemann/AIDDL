package org.aiddl.core.java.tools;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A static singleton class that allows to measure time.
 * 
 * @author Uwe Koeckemann
 */
public class StopWatch {

	/**
	 * If false start() and stop() will not do anything
	 */
	public static boolean on = true;
	/**
	 * If true all entries will be kept (rather than just their sums)
	 */
	public static boolean keepAllTimes = false;
	/**
	 * If true every call to start() and stop() will be printed 
	 */
	public static boolean verbose = false;
	
	private static Map<String,Long> startTimes = new HashMap<String,Long>();
	private static Map<String,Long> numRecordedTimes = new HashMap<String,Long>();
	private static Map<String,ArrayList<Long>> recordedTimes = new HashMap<String,ArrayList<Long>>();
	
	private StopWatch() {}
	
	/**
	 * Start a timer by recording its start time
	 * @param name The name of the timer
	 */
	public static void start(String name) {
		long t = System.currentTimeMillis();
		startTimes.put(name, t);
		if ( verbose ) System.out.println("<START> "+name);
	}
	
	/**
	 * Stop a timer
	 * @param name The name of the timer
	 * @return Elapsed time in milliseconds since timer was started
	 */
	public static long stop(String name) {
		long t = System.currentTimeMillis();
		if ( on ) {
			Long d = t - startTimes.get(name).longValue();
			if ( !recordedTimes.containsKey(name) ) {
				recordedTimes.put(name,new ArrayList<Long>());
				numRecordedTimes.put(name, Long.valueOf(0));
				
			}
			
			if ( keepAllTimes ) {
				recordedTimes.get(name).add(d);
			} else {
				long prev = 0;
				if ( !recordedTimes.get(name).isEmpty() ) {
					prev = recordedTimes.get(name).get(0);
					recordedTimes.get(name).remove(0);					
				}
				recordedTimes.get(name).add(d + prev);
			}
			
			numRecordedTimes.put(name, Long.valueOf(numRecordedTimes.get(name).longValue()+1));
			
			if ( verbose )
				System.out.println("<STOP> "+name);
			return d.longValue();
		}
		if ( verbose )
			System.out.println("<STOP> "+name);
		return 0;
	}
	/**
	 * Get a {@link List} of names of all timers.
	 * @return A {@link List} of {@link String}s containing all timers that were used.
	 */
	public static List<String> getKeyNames() {
		ArrayList<String> r = new ArrayList<String>();
		r.addAll(recordedTimes.keySet());
		return r;
	}

	/**
	 * Get average time (in ms) of all records for a timer
	 * @param name Name of the timer
	 * @return Average of all entries of requested timer (in ms)
	 */
	public static double getAvg(String name) {
		double sum = 0.0;
		
		for ( Long d : recordedTimes.get(name) ) {
			sum += d.doubleValue();
		}
		return sum/(numRecordedTimes.get(name).doubleValue());		
	}
	/**
	 * Get sum of times (in ms) of all records for a timer
	 * @param name Name of the timer
	 * @return Sum of all entries of requested timer (in ms)
	 */
	public static long getSum(String name) {
		long sum = 0;
		
		for ( Long d : recordedTimes.get(name) ) {
			sum += d.longValue();
		}
		return sum;		
	}
	/**
	 * Get latest recorded time (in ms) of a timer
	 * @param name Name of the timer
	 * @return Latest entries of requested timer (in ms)
	 */
	public static long getLast(String name) {
		return recordedTimes.get(name).get(recordedTimes.get(name).size()-1).longValue();
	}
	/**
	 * Get latest recorded time as a {@link String} formated in seconds
	 * @param name Name of the timer
	 * @return Latest entries of requested timer in seconds formated as a {@link String}
	 */
	public static String getLastFormattedInSeconds(String name) {
		if ( !recordedTimes.containsKey(name) ) {
			return "-";
		}
		double s = recordedTimes.get(name).get(recordedTimes.get(name).size()-1).doubleValue()/1000.0;
		
		return String.format(name + ": %.2fs", s);
	}
	/**
	 * Get all entries of a timer in seconds as a {@link String} separated by commas
	 * @param name Name of the timer
	 * @return A comma-separated {@link String} of {@link Double} values in seconds
	 */
	public static String getCSVFormattedInSeconds(String name) {
		if ( !recordedTimes.containsKey(name) ) {
			return "-";
		}
		
		String csvStr = "";
		for ( Long d : recordedTimes.get(name) ) {
			csvStr += "," + (d.doubleValue()/1000.0);
		}
		
		return csvStr.substring(1);
	}
	/**
	 * Reset internal data structure
	 */
	public static void reset() {
		startTimes = new HashMap<String,Long>();
		recordedTimes = new HashMap<String,ArrayList<Long>>();
		
	}
		
	/**
	 * Get a string representation of the sums of all timers in seconds
	 * @return A {@link String} showing the sum of all timer entries and the number of entries
	 */
	public static String allSums2Str() {
		String s = "";
		ArrayList<String> keys = new ArrayList<String>();
		keys.addAll(recordedTimes.keySet());
		Collections.sort(keys);
		
		for ( String k : keys ) {
			s += String.format(k + ": %.2fs (%d recorded times)\n", getSum(k)/1000.0, numRecordedTimes.get(k).longValue());
		}
		return s;
	}
	/**
	 * Get a string representation of the latest values of all timers in seconds
	 * @return A {@link String} showing the sum of all timer entries and the number of entries
	 */
	public static String allLast2Str() {
		String s = "=== StopWatch (last) ===\n";
		
		ArrayList<String> keys = new ArrayList<String>();
		keys.addAll(recordedTimes.keySet());
		Collections.sort(keys);
		
		for ( String k : keys ) {
			s += String.format(k + ": %.2fs\n", ((double)recordedTimes.get(k).get(recordedTimes.get(k).size()-1))/1000.0);
		}
		return s;
	}
	
//	public StopWatch copy() {
//		StopWatch c = new StopWatch();
//		c.addRecords(this);
//		return c;
//	}
}
