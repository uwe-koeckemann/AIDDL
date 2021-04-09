package org.aiddl.core.representation;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;

/** 
 * Abstract super class to all AIDDL data. Also used as a factory to create AIDDL objects.
 * <p> 
 * Contains many duck type methods that are only overwritten by few concrete AIDDL classes.
 * This is to make data access easy without casting. Concrete types can be safely assumed
 * if AIDDL definitions are met. If an unsupported method is used, an 
 * {@link IllegalAccessError} is thrown.
 * <p>
 * All terms are constructed through factory methods in this class.
 *  
 * @author Uwe Koeckemann
 *
 */
public abstract class Term {
	
	Term() {};
	
	private static Map<String, SymbolicTerm> symbol_table = new HashMap<>();
	public static Set<TupleTerm> tuples = new HashSet<>();
	
	/** 
	 * Get a Boolean term.
	 * @param value Boolean value 
	 * @return a Boolean term
	 */
	public static BooleanTerm bool( Boolean value ) {
		return value ? BooleanTerm.TRUE : BooleanTerm.FALSE;
	}
	
	/** 
	 * Create a symbolic term.
	 * @param value String representing the symbol
	 * @return a symbolic term
	 */
	public static SymbolicTerm sym( String value ) {
		if ( value.toLowerCase().equals("true") )
			return BooleanTerm.TRUE;
		else if ( value.toLowerCase().equals("false") )
			return BooleanTerm.FALSE;
		
		SymbolicTerm x = symbol_table.get(value);
		if ( x != null ) {
			return x;
		}
		x = new SymbolicTerm(value);
		symbol_table.put(value, x);
		
		return x;
	}	
	
	/** 
	 * Create a string term.
	 * @param value the string
	 * @return a string term
	 */
	public static StringTerm string( String value ) {
		return new StringTerm(value);
	}
	
	/** 
	 * Create an integer term.
	 * @param value integer value
	 * @return an integer term
	 */
	public static IntegerTerm integer( Integer value ) {
		return new IntegerTerm(Long.valueOf(value));
	}
	
	/**
	 * Create an integer term.
	 * @param value long value
	 * @return an integer term
	 */
	public static IntegerTerm integer( Long value ) {
		return new IntegerTerm(value);
	}
	
	/** 
	 * Create a rational term from two long values.
	 * @param p the numerator
	 * @param q the denominator
	 * @return a rational term
	 */
	public static RationalTerm rational( Long p, Long q) {
		return new RationalTerm(p, q);
	}
	
	/** 
	 * Create a rational term from two integer values.
	 * @param p the numerator
	 * @param q the denominator
	 * @return a rational term
	 */
	public static RationalTerm rational( Integer p, Integer q) {
		return new RationalTerm(p, q);
	}
	
	/** 
	 * Create a real term from a double value.
	 * @param value the double value
	 * @return a real term
	 */
	public static RealTerm real( Double value ) {
		return new RealTerm(value);
	}
	
	/** 
	 * Create a term representing positive infinity.
	 * @return an positive infinity term
	 */
	public static InfinityTerm infPos() {
		return InfinityTerm.Positive;
	}
	
	/** 
	 * Create a term representing positive or negative infinity.
	 * @param isNegative flag indicating if the term is positive or negative.
	 * @return an infinity term
	 */
	public static InfinityTerm infNeg() {
		return InfinityTerm.Negative;
	}
	
	/** 
	 * Create a variable.
	 * @param variable string representation of variable ('?' prefix not required) 
	 * @return a variable term
	 */
	public static VariableTerm var( String variable ) {
		return new VariableTerm(variable);
	}
	
	/** 
	 * Create a variable with a module name.
	 * @param variable string representation of variable ('?' prefix not required) 
	 * @param moduleName string representation of module name
	 * @return a variable term
	 */
	public static VariableTerm var( String variable, String moduleName ) {
		return new VariableTerm(variable, moduleName);
	}
	/** 
	 * Create an anonymous variable.
	 * @return a unique variable term not expected to be used anywhere else
	 */
	public static VariableTerm anonymousVar( ) {
		return new VariableTerm();
	}
	/** 
	 * Create a reference with a name and a module. 
	 * @param refName name of an {@link Entry}
	 * @param modName name of the module in which the {@link Entry} can be found
	 * @return a reference term
	 */
	public static ReferenceTerm ref( Term refName, Term modName ) {
		return new ReferenceTerm(refName, modName);
	}
	
	/**
	 * Create a reference including a local name.
	 * @param refName name of an {@link Entry}
	 * @param localName local alias used for the module in which the entry can be found
	 * @param modName name of the module in which the {@link Entry} can be found
	 * @return a reference term
	 */
	public static ReferenceTerm ref( Term refName, Term localName, Term modName ) {
		return new ReferenceTerm(refName, localName, modName );
	}
	
	/** 
	 * Create a function reference with a name and a module. 
	 * @param funName name of a function
	 * @param funModName name of the module in which the function can be found
	 * @return a function reference term
	 */
	public static FunctionReferenceTerm fref( SymbolicTerm funName, SymbolicTerm funModName, FunctionRegistry freg ) {
		return new FunctionReferenceTerm(funName, funModName, freg);
	}
	
	/** 
	 * Create a function reference with a name and a module. 
	 * @param funURI URI of a function
	 * @return a function reference term
	 */
	public static FunctionReferenceTerm fref( SymbolicTerm funURI, FunctionRegistry freg ) {
		return new FunctionReferenceTerm(funURI, freg);
	}
	
	/** 
	 * Create a function reference with a name and a module. 
	 * @param funURI URI of a function
	 * @return a function reference term
	 */
	public static FunctionReferenceTerm fref( ReferenceTerm funRef, FunctionRegistry freg ) {
		return new FunctionReferenceTerm(funRef.getRefTarget().asSym(), funRef.getRefModule().asSym(), freg);
	}
	
	/** 
	 * Create a function reference with a name and a module. 
	 * @param funURI URI of a function
	 * @return a function reference term
	 */
//	public static FunctionReferenceTerm fref( TupleTerm lambdaExp, FunctionRegistry freg ) {
//		return new FunctionReferenceTerm(lambdaExp, freg);
//	}
	
	/** 
	 * Create a function reference directly from a function. 
	 */
//	public static FunctionReferenceTerm fref( Term ref, Function f, FunctionRegistry freg ) {
//		return new FunctionReferenceTerm(ref, f, freg);
//	}
	
	/** 
	 * Create a key-value term.
	 * @param key the key
	 * @param value the value
	 * @return a key value term
	 */
	public static KeyValueTerm keyVal( Term key, Term value ) {
		return new KeyValueTerm(key,value);
	}
	
	/** 
	 * Create a tuple term from an array of terms.
	 * @param values an array of terms
	 * @return a tuple term
	 */
	public static TupleTerm tuple( Term... values ) {
		return new TupleTerm(values);
	}
	/** 
	 * Create a tuple from a list of terms.
	 * @param values list of value terms
	 * @return a tuple 
	 */
	public static TupleTerm tuple( List<? extends Term> values ) {
		return new TupleTerm(values);
	}
	
	/** 
	 * Create a tuple from a list of terms.
	 * @param values list of value terms
	 * @return a tuple 
	 */
	public static TupleTerm tuple( LockableList values ) {
		return new TupleTerm(values);
	}
	
	/** 
	 * Create a list term from an array of terms.
	 * @param values an array of terms
	 * @return a list term
	 */
	public static ListTerm list( Term... values ) {
		if ( values.length == 0 ) {
			return ListTerm.Empty;
		}
		return new ListTerm(values);
	}
	
	/**
	 * Create a list term from a list of terms.
	 * @param values a list of terms
	 * @return a list term
	 */
	public static ListTerm list( Collection<? extends Term> values ) {
		if ( values.isEmpty() ) {
			return ListTerm.Empty;
		}
		return new ListTerm(values);
	}
	
	/** 
	 * Create a tuple from a list of terms.
	 * @param values list of value terms
	 * @return a tuple 
	 */
	public static ListTerm list( LockableList values ) {
		return new ListTerm(values);
	}
	
	/** 
	 * Create a list term from a map from term to term.
	 * @param values a map from term to term
	 * @return a list term containing a key-value term for each entry in the map
	 */
	public static ListTerm list( Map<Term,Term> values ) {
		if ( values.isEmpty() ) {
			return ListTerm.Empty;
		}
		return new ListTerm(values);
	}
		
	/** 
	 * Create a set term from an array of terms.
	 * @param values an array of terms
	 * @return a set term
	 */
	public static SetTerm set(  Term... values ) {
		if ( values.length == 0 ) {
			return SetTerm.Empty;
		}
		Collection<Term> new_values = new LinkedHashSet<>();
		for ( Term t : values ) {
			new_values.add(t);
		}
		return new SetTerm(new_values);
	}	
	
	/** 
	 * Create a set term from a collection of terms.
	 * @param values a collection of terms
	 * @return a set term
	 */
	public static SetTerm set( Collection<? extends Term> values ) {
		if ( values.isEmpty() ) {
			return SetTerm.Empty;
		}
		return new SetTerm(values);
	}
	
	/** 
	 * Create a set term from a collection of terms.
	 * @param values a collection of terms
	 * @return a set term
	 */
	public static SetTerm set( LockableSet values ) {
		return new SetTerm(values);
	}
	
	/** 
	 * Create a set term from a map from term to term.
	 * @param values a map from term to term
	 * @return a set term containing a key-value term for each entry in the map
	 */
	public static SetTerm set( Map<Term,Term> values ) {
		if ( values.isEmpty() ) {
			return SetTerm.Empty;
		}
		return new SetTerm(values);
	}	
	
	/**
	 * Match a term to another term.
	 * @param t 
	 * @return A substitution to make both terms equal.
	 */
	public Substitution match( Term t ) {
		if ( this.equals(t) ) {
			return new Substitution();
		}
		return null;
	}
	
	/**
	 * Perform a substitution. Recursively replaces all variable terms that appear in substitution.  
	 * @param s Substitution to be applied
	 * @return Term resulting of applying substitution. 
	 */
	public abstract Term substitute( Substitution s );
	
	/** 
	 * Recursively resolve all references in a term.
	 * <b>Note:</b> Use with caution. This may lead to infinite recursion, e.g., in case of recursive
	 * type definitions. This can be avoided by processing a term recursively while only resolving 
	 * references as they come up.
	 * @param db a container used to resolve references
	 * @return a term in which all references are resolved
	 */
	public abstract Term resolve( Container db );
	
	/**
	 * Checks if this {@link Term} is ground. Ground {@link Term}s are either constants or complex {@link Term}s containing no variables.
	 * @return <code>true</code> if {@link Term} is ground, <code>false</code> otherwise.
	 */
	public abstract boolean isGround();
	
	/**
	 * Checks if this {@link Term} is complex.
	 * @return <code>true</code> if {@link Term} is complex, <code>false</code> otherwise.
	 */
	public abstract boolean isComplex();

	/**
	 * Checks if this {@link Term} is a variable.
	 * @return <code>true</code> if {@link Term} is variable, <code>false</code> otherwise.
	 */
	public abstract boolean isVariable();

	/**
	 * Checks if this {@link Term} is a constant.
	 * @return <code>true</code> if {@link Term} is constant, <code>false</code> otherwise.
	 */
	public abstract boolean isConstant();
		
//	private Term throwHelper( Term arg ) {
//		throw new IllegalAccessError(String.format("(%s) %s", arg.getClass().getSimpleName(), arg.toString()));
//	}
	private <T extends Term> T throwHelper( Term arg1, Term arg2 ) {
		throw new IllegalAccessError(String.format("%s (%s) X %s (%s)", arg1.toString(), arg1.getClass().getSimpleName(), arg2.toString(), arg2.getClass().getSimpleName()));
	}
	
	/** 
	 * Get boolean value of this term.
	 * @return boolean value 
	 */
	public Boolean getBooleanValue() { throw new IllegalAccessError(this.toString()); };
	/** 
	 * Get this term's numerator. Assumes term is and instance of  {@link IntegerTerm} or {@link RationalTerm}.
	 * @return long value of rational number numerator
	 */
	public Long getNumerator() { throw new IllegalAccessError(this.toString()); }
	/** 
	 * Get this term's denominator. Assumes term is and instance of  {@link IntegerTerm} or {@link RationalTerm}.
	 * @return long value of rational number denominator
	 */
	public Long getDenominator() { throw new IllegalAccessError(this.toString()); }
	/** 
	 * Get this term's long value. Assumes term is an instance of {@link IntegerTerm}.
	 * @return long value of integer term
	 */
	public Long getLongValue() { throw new IllegalAccessError(this.toString()); }
	/** 
	 * Get this term's integer value. Assumes term is an instance of {@link IntegerTerm}.
	 * @return integer value of integer term
	 */
	public Integer getIntValue() { throw new IllegalAccessError(this.toString()); }
	/** 
	 * Get this term's double value.  Assumes term is an instance of {@link IntegerTerm}, {@link RationalTerm}, or {@link RealTerm}.
	 * @return integer value of integer term
	 */
	public Double getDoubleValue() { throw new IllegalAccessError(this.toString()); }
	/** 
	 * Get this term's string value
	 * @return string value of this term
	 */
	public String getStringValue() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get this term's key. Assumes instance of {@link KeyValueTerm}.
	 * @return key of {@link KeyValueTerm}
	 */
	public Term getKey() {	throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get this term's value. Assumes instance of {@link KeyValueTerm}.
	 * @return value of {@link KeyValueTerm}
	 */
	public Term getValue() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get number of elements in this term. Assumes instance of {@link TupleTerm} or {@link CollectionTerm}.
	 * @return the number of terms contained in this term
	 */
	public Integer size() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Access the nth element in a term. Assumes instance of {@link TupleTerm} or {@link ListTerm}.
	 * @param n index to access
	 * @return nth element of term
	 */
	public Term get( int n ) { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get value from a key. Looks for instances of {@link KeyValueTerm} in a {@link TupleTerm}, {@link CollectionTerm}, or directly uses a {@link KeyValueTerm} 
	 * (in the latter case this is the same as getValue()).
	 * @param key a key
	 * @return value if key exists, <code>null</code> otherwise
	 */
	public Term get( Term key ) { return throwHelper(this, key); }
	
	/** 
	 * Get value from a key or a default value. Looks for instances of {@link KeyValueTerm} in a {@link TupleTerm}, {@link CollectionTerm}, or directly uses a {@link KeyValueTerm} 
	 * (in the latter case this is the same as getValue()). If this method would fail for any reason it returns the default value instead.
	 * @param key a key
	 * @param def a term returned as default value
	 * @return value if key exists, default otherwise
	 */
	public Term getOrDefault( Term key, Term def ) { return def; }
	
	/** 
	 * Get value from a key or a default value. Looks for instances of {@link KeyValueTerm} in a {@link TupleTerm}, {@link CollectionTerm}, or directly uses a {@link KeyValueTerm} 
	 * (in the latter case this is the same as getValue()). If this method would fail for any reason it returns the default value instead.
	 * @param key a key
	 * @param def a term returned as default value
	 * @return value if key exists, default otherwise
	 */
	public Term getOrPanic( Term key ) {
		Term r = this.get(key);
		if ( r == null ) {
			throw new IllegalAccessError("Term " + this + " does not have key " + key + " (use get or getOrDefault to avoid exceptions).");
		}
		return r;
	}
	
	/** 
	 * Get the target of a reference. Assumes instance of {@link ReferenceTerm}.
	 * @return reference target
	 */
	public Term getRefTarget() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get the alias of a reference. Assumes instance of {@link ReferenceTerm}.
	 * @return reference alias
	 */
	public Term getRefAlias() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * Get the module referred to by a reference. Assumes instance of {@link ReferenceTerm}.
	 * @return referred module
	 */
	public Term getRefModule() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 *If this term is a function reference view it as a such.
	 * @return referred module
	 */
	public FunctionReferenceTerm asFunRef() { throw new IllegalAccessError(this.toString()); }

	/**
	 * If this term is symbolic view it as a SymbolicTerm.
	 * @return symbolic term
	 */
	public SymbolicTerm asSym()  { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is a collection, view it as a collection.  
	 * @throws IllegalAccessError when operation is not supported.
	 * @return collection term
	 */
	public CollectionTerm asCollection() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is a set, view it as a set.
	 * @throws IllegalAccessError when operation is not supported.
	 * @return set term
	 */
	public SetTerm asSet() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is a list, view it as a list. 
	 * @throws IllegalAccessError when operation is not supported.
	 * @return list term
	 */
	public ListTerm asList() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is a tuple, view it as a tuple term.
	 * @return tuple term
	 */
	public TupleTerm asTuple() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is a reference, view it as a reference term
	 * @return tuple term
	 */
	public ReferenceTerm asRef() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is numerical view it as a numerical term
	 * @return tuple term
	 */
	public NumericalTerm asNum() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is integer view it as a integer term
	 * @return tuple term
	 */
	public IntegerTerm asInt() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is rational view it as a rational term
	 * @return tuple term
	 */
	public RationalTerm asRational() { throw new IllegalAccessError(this.toString()); }
	
	/** 
	 * If this term is real view it as a real term
	 * @return tuple term
	 */
	public RealTerm asReal() { throw new IllegalAccessError(this.toString()); }
	
	
	/** 
	 * Get a {@link Collection} copied from collection term.
	 * @return a {@link Collection} of terms
	 */
	public Collection<Term> getCollectionCopy() { throw new IllegalAccessError(this.toString()); }

	/** 
	 * Get a {@link Set} copied from collection term
	 * @return a {@link Set} of terms
	 */
	public Set<Term> getSetCopy() { throw new IllegalAccessError(this.toString()); }

	/** 
	 * Get a {@link List} copied from collection term
	 * @return a {@link List} of terms
	 */
	public List<Term> getListCopy() { throw new IllegalAccessError(this.toString()); }
	
	@Override
	public abstract String toString();
	
    @Override
	public abstract int hashCode();
    
    @Override
    public abstract boolean equals( Object o );
}