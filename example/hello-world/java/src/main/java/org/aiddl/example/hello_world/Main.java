package org.aiddl.example.hello_world;

import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.StringTerm;

public class Main {
    public static void main( String[] args ) {
        StringTerm helloAiddl = Term.string("Hello AIDDL World!");
        System.out.println(helloAiddl);
    }
}
