package org.aiddl.core.java.tools;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Provides an {@link Iterator} over a combination of choices of class T.
 * <p>
 * A memory friendly, generic combo builder.
 *
 * @author Uwe Koeckemann
 *
 * @param <T>
 */
public class ComboIterator<T> implements Iterator<List<T>>, Iterable<List<T>> {
	private int [] slotSizes;
	private int [] slots; 
	private boolean carry = false;
	private boolean done = false;
	
	List<List<T>> in;

	/**
	 * Create iterator by providing choices. 
	 * @param toCombine list of lists of choices
	 */
	public ComboIterator( List<List<T>> toCombine ) {
		this.in = toCombine;
		
//		this.slots = new int[this.in.size()];
//		this.slotSizes = new int[this.in.size()];
//		for ( int i = 0 ; i < this.in.size() ; i++ ) {
//			slotSizes[i] = this.in.get(i).size();
//		}
		
		if ( this.in.isEmpty() ) {
			done = true;
			return;
		}
		for ( List<T> choices : this.in ) {
			if ( choices.isEmpty() ) {
				done = true;
				return;
			}
		}
		
		this.slots = new int[this.in.size()];
		this.slotSizes = new int[this.in.size()];
		for ( int i = 0 ; i < this.in.size() ; i++ ) {
			slotSizes[i] = this.in.get(i).size();
		}
	}
	
	/**
	 * Calculates and returns the number of combinations of the input choices.
	 * @return the number of combinations
	 */
	public int getNumCombos() {
		int n = 1;
		for ( int i = 0 ; i < slotSizes.length ; i++ ) {
			n *= slotSizes[i];
		}
		return n;
	}

	@Override
	public boolean hasNext() {
		return !done;
	}

	@Override
	public List<T> next() {
		List<T> combo = new ArrayList<T>(slots.length);
		for ( int i = 0 ; i < slots.length ; i++ ) {
			combo.add(in.get(i).get(slots[i]));
		}
		this.iterate();
		return combo;
	}

	@Override
	public void remove() {
		throw new UnsupportedOperationException();
	}
	
	/**
	 * Setup next combination
	 */
	private void iterate() {
		
		if ( done ) {	// reinit
			done = false;
			carry = false;
			for  ( int i = 0 ; i < slots.length; i++ ) {
				slots[i] = 0;
			}
		}
		
		carry = false;
		
		slots[0]++;
		
		for ( int i = 0 ; i < slots.length; i++ ) {
			if ( carry ) {
				slots[i]++;
				carry = false;
			}
			
			if ( slots[i] == slotSizes[i] ) {
				slots[i] = 0;
				carry = true;
			} 
		}
		
		if ( carry ) {
			done = true;
		} 
		
	}

	@Override
	public Iterator<List<T>> iterator() {
		return this;
	}
}
