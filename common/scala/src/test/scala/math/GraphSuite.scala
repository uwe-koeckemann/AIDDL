import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.graph.BellmanFord
import org.aiddl.common.scala.math.graph.StronglyConnectedComponentExtractor
import org.aiddl.common.scala.math.graph.AdjacencyListGraph
import org.aiddl.common.scala.math.graph.PathExtractor

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.function.DefaultFunctionUri
import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation.TermParseImplicits.string2term

class GraphSuite extends AnyFunSuite {
    test("Path exists") {
        val c = new Container()
        val m = Parser.parseInto("../test/math/graph/bellman-ford.aiddl", c)
        var g = c.getEntry(m, Sym("G")).get.v
        g = c.resolve(g)
        val w = c.eval(c.resolve(g(Weights)))

        val fBF = new BellmanFord
        val graph = new AdjacencyListGraph(g)
        fBF(graph, "a", w)

        val fPath = new PathExtractor
        val path = fPath("a", "f", fBF.pi(_))

        assert( path == Parser.str("[a b c f]"))
    }   
    
    test("No path exists") {
        val c = new Container()
        Function.loadDefaultFunctions(c)

        val fBF = new BellmanFord

        val w = c.eval("(org.aiddl.eval.lambda ?x 1)")
        val g = "(V : {a b c d e f} E:{ (a b) (a d) (b e) (c e) (c f) (d b) (e d) })"
        val graph = new AdjacencyListGraph(g)

        fBF(graph, "f", w)
        val fPath = new PathExtractor
        val path = fPath("a", "f", fBF.pi(_))
        assert( path == NIL )

        val pathEmpty = fPath("f", "f", fBF.pi(_))
        assert( pathEmpty == ListTerm.empty )
     }

    test("Strongly connected components") {
        val g = "(V : {a b c d e f} E : {(a b) (a d) (b e) (c e) (c f) (d b) (e d) (f f)})"

        val scc_true = Parser.str("{{c} {f} {a} {b d e}}")
        val scc = new StronglyConnectedComponentExtractor()

        val scc_r = scc(g)
        assert(scc_r == scc_true)
    }
}