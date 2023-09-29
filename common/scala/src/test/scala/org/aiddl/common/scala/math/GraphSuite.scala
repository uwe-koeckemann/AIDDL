package org.aiddl.common.scala.math

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, BellmanFord, PathExtractor, StronglyConnectedComponentExtractor}
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.{DefaultFunctionUri, Function}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class GraphSuite extends AnyFunSuite {
    test("Path exists") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/math/graph/bellman-ford.aiddl")
        assert(c.typeCheckModule(m))
        var g = c.getProcessedValueOrPanic(m, Sym("G"))
        g = c.resolve(g)
        val w = c.eval(c.resolve(g(Weights)))

        val fBF = new BellmanFord
        val graph = new AdjacencyListGraph(g)
        fBF(graph, Sym("a"), w)

        val fPath = new PathExtractor
        val path = fPath(Sym("a"), Sym("f"), fBF.pi(_))

        assert( path == parser.str("[a b c f]"))
    }   
    
    test("No path exists") {
        val c = new Container()
        val parser = new Parser(c)
        Function.loadDefaultFunctions(c)

        val fBF = new BellmanFord

        val w = c.eval(parser.str("(org.aiddl.eval.lambda ?x 1)"))
        val g = parser.str("(V : {a b c d e f} E:{ (a b) (a d) (b e) (c e) (c f) (d b) (e d) })")
        val graph = new AdjacencyListGraph(g)

        fBF(graph, Sym("f"), w)
        val fPath = new PathExtractor
        val path = fPath(Sym("a"), Sym("f"), fBF.pi(_))
        assert( path == NIL )

        val pathEmpty = fPath(Sym("f"), Sym("f"), fBF.pi(_))
        assert( pathEmpty == ListTerm.empty )
     }

    test("Strongly connected components") {
        val c = new Container
        val parser = new Parser(c)
        val g = parser.str("(V : {a b c d e f} E : {(a b) (a d) (b e) (c e) (c f) (d b) (e d) (f f)})")

        val scc_true = parser.str("{{c} {f} {a} {b d e}}")
        val scc = new StronglyConnectedComponentExtractor()

        val scc_r = scc(g)
        assert(scc_r == scc_true)
    }
}