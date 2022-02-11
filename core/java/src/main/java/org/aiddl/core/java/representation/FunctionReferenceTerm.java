package org.aiddl.core.java.representation;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;

/** A term representing a reference to a function
 * @author Uwe Koeckemann
 *
 */
public class FunctionReferenceTerm extends Term {
	FunctionRegistry freg;
	Function f;
	SymbolicTerm ref;
	
	private FunctionReferenceTerm() {
	}
	
//	protected FunctionReferenceTerm( Term ref, Function f, FunctionRegistry freg ) {
//		this.freg = freg;
//		this.f = f;
//		this.ref = ref;
//	}
	
	protected FunctionReferenceTerm( SymbolicTerm funName, SymbolicTerm funModuleName, FunctionRegistry freg ) {
		this.freg = freg;
		ref = funModuleName.concat(funName);
		this.f = freg.getFunction(this.ref);
	}
	
	protected FunctionReferenceTerm( SymbolicTerm fullSymRef, FunctionRegistry freg ) {
		this.freg = freg;
		this.ref = fullSymRef;
		this.f = freg.getFunction(this.ref);
	}
	
//	protected FunctionReferenceTerm( TupleTerm lambdaExp, FunctionRegistry freg ) {
//		this.freg = freg;
//		this.ref = lambdaExp;
//		this.f = freg.getFunction(this.ref);
//	}
	
	public SymbolicTerm getFunRefTerm() {
		return ref;
	}
	
	public Function getFunction () {
		if ( this.f == null ) {
			this.f = this.freg.getFunction(this.ref);
		}
		return f;
	}

	public Function getFunctionOrPanic () {
		if ( this.f == null ) {
			this.f = this.freg.getFunctionOrPanic(this.ref);
		}
		return f;
	}
	
	@Override
	public FunctionReferenceTerm asFunRef() {
		return this;
	}
				
	@Override
	public Substitution match(Term t) {
		if ( t instanceof FunctionReferenceTerm ) {
			return this.ref.match(((FunctionReferenceTerm)t).ref);
		}
		return super.match(t);
	}	
	
	@Override
	public Term substitute(Substitution s) {
		if ( !s.isForced() ) {
			return this;
		}
		FunctionReferenceTerm frefSubbed = new FunctionReferenceTerm();
		frefSubbed.ref = this.ref.substitute(s).asSym();
		frefSubbed.freg = this.freg;
		frefSubbed.f = this.freg.getFunction(frefSubbed.ref);
		return frefSubbed;
	}
	
	@Override
	public FunctionReferenceTerm resolve(Container db) {
		FunctionReferenceTerm frefResolved = new FunctionReferenceTerm();
		frefResolved.ref = this.ref.resolve(db);
		frefResolved.freg = this.freg;
		frefResolved.f = this.freg.getFunction(frefResolved.ref);
		return frefResolved;
	}
	
	@Override
	public boolean isGround() {
		return ref.isGround();
	}
	
	@Override
	public boolean isComplex() {
		return ref.isComplex();
	}

	@Override
	public boolean isVariable() {
		return false;
	}

	@Override
	public boolean isConstant() {
		return this.ref.isConstant();
	}

	@Override
	public String toString() {
		return "^" + ref.toString();
	}

	@Override
	public int hashCode() {
		return 17*ref.hashCode();
	}

    @Override
    public boolean equals( Object o ) { 
    	if ( this == o ) {
    		return true;
    	}
    	if ( !(o instanceof FunctionReferenceTerm) ) {
    		return false;
    	}
    	FunctionReferenceTerm t = (FunctionReferenceTerm)o;
    	return this.ref.equals(t.ref); 
    }
}
