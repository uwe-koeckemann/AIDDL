package org.aiddl.common.java.game;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.aiddl.common.java.planning.state_variable.OperatorStateConstrainedDomainEnumerator;
import org.aiddl.common.java.planning.state_variable.StateTransitionFunction;
import org.aiddl.common.java.search.adversarial.MiniMaxMultiPlayer;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestTicTacToe extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	public void testAdvanceState() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		Evaluator eval = (Evaluator)fReg.getFunction(Uri.EVAL);
		Random rand = new Random();
		StateTransitionFunction trans = new StateTransitionFunction();

		Parser.parseFile(
				"../test/search/adversarial/tic-tac-toe/tic-tac-toe.aiddl", 
				db, 
				fReg);
		
//		Parser.parseFile(
//				"../test/game/tic-tac-toe/test-01.aiddl", 
//				db, 
//				fReg);
				
				
		Logger.addPrintStream(System.out);
		
		Term game = db.getEntry(Term.sym("game")).getValue();
		
		Function f_gameOver = fReg.getFunction(Term.sym("org.aiddl.common.test.search.adversarial.tic-tac-toe.game-over"));
		Function f_score = fReg.getFunction(Term.sym("org.aiddl.common.test.search.adversarial.tic-tac-toe.score-gen"));
		
		Term state = eval.apply(game.get(Term.sym("initial-state")));
//		state = db.getEntry(Term.sym("org.aiddl.test.game.tic-tac-toe.t01"), Term.sym("initial-state")).getValue();
		
		
		Term domains = eval.apply(game.get(Term.sym("domains")));
		Term player_actions = eval.apply(game.get(Term.sym("actions")));
//		ListTerm round =  eval.apply(game.get(Term.sym("round"))).asList();
		
		OperatorStateConstrainedDomainEnumerator opEnum = new OperatorStateConstrainedDomainEnumerator(domains.asSet(), eval);
		
//		MiniMax minimax = new MiniMax(f_score, f_gameOver, trans, opEnum);
//		MiniMaxAlphaBetaPruning minimax = new MiniMaxAlphaBetaPruning(f_score, f_gameOver, trans, opEnum);
		MiniMaxMultiPlayer minimax = new MiniMaxMultiPlayer(f_score, f_gameOver, trans, opEnum);
		
		
		int roundCount = 0;
		boolean gameOver = false;
		while ( !gameOver ) {
			roundCount++;
//			System.out.println("Round: " + roundCount);
			
//			for ( int i = 0 ; i < round.size() ; i++ ) {
			Term player = state.get(Term.sym("next-player"));
			Term actions = eval.apply(player_actions.get(player));
								
//				System.out.println("Player: " + player);
//				System.out.println("State: " + state);
//				System.out.println("Actions: " + actions);
				
				Term choice;
				
				if ( state.get(Term.sym("next-player")).equals(Term.sym("X")) ) {
					choice = minimax.apply(Term.tuple(state, actions));
				} else {
					SetTerm feasible = opEnum.apply(Term.tuple(state, actions)).asSet();
					List<Term> choices = new ArrayList<>();
					for ( Term e : feasible ) {
						choices.add(e);
					}
					choice = choices.get(rand.nextInt(choices.size()));
				}
				
//				System.out.println("Selected: " + choice);
				
				state = trans.apply(Term.tuple(state, choice));
				
//				System.out.println("New state: " + Logger.prettyPrint(state, 1));
				
//				System.out.println(f_gameOver.apply(state));
				
				gameOver = f_gameOver.apply(state).getBooleanValue();
				if ( gameOver ) {
					break;
				}
				
//			}
		}
		
//		System.out.println("Winner: " + state.get(Term.sym("winner")));
		
		assertTrue( state.get(Term.sym("winner")).equals(Term.sym("X")) );
	}
}
