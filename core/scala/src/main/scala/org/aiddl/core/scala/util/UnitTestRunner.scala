package org.aiddl.core.scala.util

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
import org.aiddl.core.scala.util.logger.Logger

import java.util.logging.Level

/**
 * Run AIDDL unit tests
 */
object UnitTestRunner extends Verbose {
    /** Run all tests according to #assert typed entries in a container
     * @param c container to test
     * @param verbose true if test results should be printed
     * @return fraction of successful tests.
     */
    def run( c: Container, verbose: Boolean ): Term = {
        var numSuccess = 0
        var numTests = 0
        
        val tests = c.getMatchingEntries(Var(), Sym("#assert"), Var())

        for ( e <- tests ) {
        val test = e.value
            if ( test.isInstanceOf[ListTerm] ) {
                var i = 1;
                for ( t <- test.asList ) {
                    val pass = singleTest(e.name.toString + " " + i, t, c.eval, verbose)
                    numTests += 1
                    i += 1
                    if ( pass ) numSuccess += 1
                }
            } else {
                val pass = singleTest(e.name.toString(), test, c.eval, verbose)
                numTests += 1
                if ( pass ) numSuccess += 1
            }
        }

        Num(numSuccess, numTests)
    }

    /** Run all tests in a list of AIDDL module files.
     * @param fNames filenames of modules
     * @return true if all tests pass, false otherwise
     */
    def testFiles( fNames: scala.List[String] ): Boolean = testFiles(fNames, new Container())

    /** Run all tests in a list of AIDDL module files.
     *
     * @param fNames filenames of modules
     * @param c container to use for evaluation
     * @return true if all tests pass, false otherwise
     */
    def testFiles( fNames: scala.List[String], c: Container ): Boolean = {
        val p = new Parser(c)
        for ( fName <- fNames ) {
            p.parseFile(fName)
        }
        val r = UnitTestRunner.run(c, true)
        r == Rational(1, 1)
    }

    private def singleTest(label: String, test: Term, eval: Evaluator, verbose: Boolean): Boolean = {
        val result = eval.apply(test)
        val passed = result match {
            case Bool(v) => v
            case _ => false
        }
        if (passed) {
            logger.info(label + ": ok >>> " + test)
        } else {
            logger.severe("---<= RESULT =>---")
            logger.severe(result.toString())
            logger.severe("---<= EVAL =>---")
            eval.logConfig(this.logger.level, this.logger.handler)
            eval.apply(test)
            eval.logConfig(level = Level.WARNING)
            logger.severe(label + ": FAILURE " + test.toString)
            logger.severe(" -> " + result.toString())
            throw new IllegalStateException("Test failed")
        }
        passed
    }
}