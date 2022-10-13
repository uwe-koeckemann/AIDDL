package org.aiddl.core.scala.tools

import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Rational
import org.aiddl.core.scala.function.DefaultFunctionUri as D
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.tools.Logger

object UnitTestRunner extends Verbose {
    def run( c: Container, eval: Evaluator, verbose: Boolean ): Term = {
        var numSuccess = 0
        var numTests = 0
        
        val tests = c.getMatchingEntries(Var(), Sym("#assert"), Var())

        for ( e <- tests ) {
        val test = e.value
        if ( test.isInstanceOf[ListTerm] ) {
            var i = 1;
            for ( t <- test.asList ) {
            val pass = singleTest(e.name.toString + " " + i, t, eval, verbose)
            numTests += 1
            i += 1
            if ( pass ) numSuccess += 1
            }
        } else {
            val pass = singleTest(e.name.toString(), test, eval, verbose)
            numTests += 1
            if ( pass ) numSuccess += 1
        }
        }

        Num(numSuccess, numTests)
    }

    def singleTest( label: String, test: Term, eval: Evaluator, verbose: Boolean ): Boolean = {
        val result = eval.apply(test)
        val passed = result match { case Bool(v) => v case _ => false }
        if ( passed ) {
            log(1, label + ": ok >>> " + test)
        } else {
            log(1, "---<= RESULT =>---")
            log(1, result.toString())
            log(1, "---<= EVAL =>---")
            eval.setVerbose(2)
            eval.apply(test)
            eval.setVerbose(0)
            log(1, label + ": FAILURE " + test.toString)
            log(1, " -> " + result.toString())
            throw new IllegalStateException("Test failed")
        }
        passed
    }

    def testFiles( fNames: scala.List[String] ): Boolean = {
        val c = new Container()
        val p = new Parser(c)
        Function.loadDefaultFunctions(c)
        for ( fName <- fNames ) {
            p.parseFile(fName)
        }
        val r = UnitTestRunner.run(c, c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator], true)
        r == Rational(1, 1)
    }

    def testFiles( fNames: scala.List[String], c: Container ): Boolean = {
        val p = new Parser(c)
        for ( fName <- fNames ) {
            p.parseFile(fName)
        }
        val r = UnitTestRunner.run(c, c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator], true)
        r == Rational(1, 1)
    }

}