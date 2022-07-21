package org.aiddl.common.java.math.graph;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestGraph extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
		
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
	
	public void testBellmanFord() {
		Container db = new Container();
		FunctionRegistry fReg = db.getFunctionRegistry();
		
		Parser.parseFile(aiddlTestStr + "/math/graph/bellman-ford.aiddl", db);

		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
			
		fReg.loadContainerDefintions(db);
			
		BellmanFord bf = new BellmanFord();
		bf.configure(null, fReg);
		
		Term G = db.getEntry(Term.sym("G")).getValue().resolve(db);
		Term w = G.get(GraphTerm.Weights);
		
		w = w.resolve(db);	
		w = eval.apply(w);	

		Term arg = Term.tuple(G, w, Term.sym("a"));

		assertTrue( eval.apply( fReg.getInputChecker(bf.getInterfaceUri()), arg ).getBooleanValue() );
		Term answer = bf.apply(arg);
		assertTrue( eval.apply( fReg.getOutputChecker(bf.getInterfaceUri()), answer ).getBooleanValue() );
//
		Term pi = answer.get(Term.sym("predecessor"));
//		
		PathExtractor pExtract = new PathExtractor();
		Term path = pExtract.apply(Term.tuple(pi, Term.sym("a"), Term.sym("f")));
//		
		assertTrue( path.equals(Parser.ParseTerm("[a b c f]")) );
	}
	
	public void testBellmanFordNoPathExists() {
		Container db = new Container();
		FunctionRegistry fReg = db.getFunctionRegistry();
		
		String graph_mod = Parser.getModuleFilename(Term.sym("org.aiddl.common.math.graph"));
		Parser.parseFile(graph_mod, db);
		
		TupleTerm G = Parser.ParseTerm(""
				+ "(V : {a b c d e f} "
				+ "E : {"
				+ "  (a b)"
				+ "  (a d)"
				+ "  (b e)"
				+ "  (c e)"
				+ "  (c f)"
				+ "  (d b)"
				+ "  (e d)"
				+ "})").asTuple();
		
		Term w = Parser.ParseTerm("(org.aiddl.eval.lambda ?x 1)");
		Term w_fun = fReg.getFunction(Uri.EVAL).apply(w);
		
		Term arg = Term.tuple(G, w_fun, Term.sym("f"));

		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
			
		fReg.loadContainerDefintions(db);
		BellmanFord bf = new BellmanFord();
		bf.configure(null,  fReg);

		assertTrue( eval.apply( fReg.getInputChecker(bf.getInterfaceUri()), arg ).getBooleanValue() );
		Term answer = bf.apply(arg);
		assertTrue( eval.apply( fReg.getOutputChecker(bf.getInterfaceUri()), answer ).getBooleanValue() );

		Term pi = answer.get(Term.sym("predecessor"));

		PathExtractor pExtract = new PathExtractor();
		Term path = pExtract.apply(Term.tuple(pi, Term.sym("a"), Term.sym("f")));

		assertTrue( path.equals(CommonTerm.NIL) );
		path = pExtract.apply(Term.tuple(pi, Term.sym("f"), Term.sym("f")));
		assertTrue( path.equals(Term.list()) );
		
		
	}
	
	public void testSCC() {
		TupleTerm G = Parser.ParseTerm(""
				+ "(V : {a b c d e f} "
				+ "E : {"
				+ "  (a b)"
				+ "  (a d)"
				+ "  (b e)"
				+ "  (c e)"
				+ "  (c f)"
				+ "  (d b)"
				+ "  (e d)"
				+ "  (f f)"				
				+ "})").asTuple();
		
		StronglyConnectedComponents scc = new StronglyConnectedComponents();
		
		Term sccResult = scc.apply(G);
		
		assertTrue( sccResult.equals(Parser.ParseTerm("{{c} {f} {a} {b d e}}")) );
		
		
	}
	
	public void testTranspose() {
		TupleTerm G = Parser.ParseTerm(""
				+ "(V : {a b c d e f} "
				+ "E : {"
				+ "  (a b)"
				+ "  (a d)"
				+ "  (b e)"
				+ "  (c e)"
				+ "  (c f)"
				+ "  (d b)"
				+ "  (e d)"
				+ "  (f f)"				
				+ "})").asTuple();
		TupleTerm G_t = Parser.ParseTerm(""
				+ "(V : {a b c d e f} "
				+ "E : {"
				+ "  (b a)"
				+ "  (d a)"
				+ "  (e b)"
				+ "  (e c)"
				+ "  (f c)"
				+ "  (b d)"
				+ "  (d e)"
				+ "  (f f)"				
				+ "})").asTuple();
		Transpose f_trans = new Transpose();
		
		Term trans_res = f_trans.apply(G);
		
		assertTrue(G_t.equals(trans_res));
		
		Term trans_res_2 = f_trans.apply(trans_res);
		
		assertTrue(G.equals(trans_res_2));
		
	}
	
	
	public void testDepthFirstSearch() {
		Container db = new Container();
		FunctionRegistry fReg = db.getFunctionRegistry();
		
		Parser.parseFile(aiddlTestStr + "/math/graph/bellman-ford.aiddl", db);

		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
			
		fReg.loadContainerDefintions(db);
		
		TupleTerm G = Parser.ParseTerm(""
				+ "(V : {a b c d e f} "
				+ "E : {"
				+ "  (a b)"
				+ "  (a d)"
				+ "  (b e)"
				+ "  (c e)"
				+ "  (c f)"
				+ "  (d b)"
				+ "  (e d)"
				+ "  (f f)"				
				+ "})").asTuple();
		
		DepthFirstSearch dfs = new DepthFirstSearch();
		
		assertTrue( eval.apply( fReg.getInputChecker(dfs.getInterfaceUri()), G ).getBooleanValue() );
		Term answer = dfs.apply(G);
		assertTrue( eval.apply( fReg.getOutputChecker(dfs.getInterfaceUri()), answer ).getBooleanValue() );
	}
}
