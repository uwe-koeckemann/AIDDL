package org.aiddl.common.scala.execution.behavior_tree

import scala.collection.mutable.HashMap
import java.io.PrintWriter
import org.aiddl.core.scala.representation.{CollectionTerm, FunRef, KeyVal, ListTerm, SetTerm, Str, Sym, Term}

class BehaviorTree2Dot extends org.aiddl.core.scala.function.Function {

    var nextID = 0;
    val nodeMap = Map( "sequence" -> "->", "selector" -> "?" )

    def toFile( x: Term, fn: String ) = {
        val s = this(x)
        new PrintWriter(fn) { write(s.asStr.value); close }
    }

    def apply( x: Term ): Term = {
        val inputs = x.getOrElse(Sym("inputs"), SetTerm.empty).asCol
        val outputs = x.getOrElse(Sym("outputs"), SetTerm.empty).asCol
        nextID = 0
        val v = new HashMap[Int, String]
        val e = new HashMap[Int, List[Int]]
        val sb = new StringBuilder
        this.getNodesAndEdges(x(Sym("behavior-tree")), v, e, inputs, outputs)

        sb.append("digraph {\n")
        v.foreach( (n, s) => sb.append(s"""\t$s\n"""))
        sb.append("\n")
        e.foreach( (n1, adjList) => { adjList.reverse.foreach( n2 => sb.append(s"""\t$n1 -> $n2\n""") ) })
        sb.append("}")

        Str(sb.toString)
    }

    def getNodesAndEdges( x: Term, nodes: HashMap[Int, String], edges: HashMap[Int, List[Int]], inputs: CollectionTerm, outputs: CollectionTerm ): Int = {
        val myNode = nextID; 
        this.nextID += 1
        x match {
            case KeyVal(Sym("leaf"), t) => {
                val inStr = if inputs.containsKey(t) then s"${inputs(t)} " else ""
                val outStr = if outputs.containsKey(t) then s" ${outputs(t)}" else ""

                val shape =
                    if inputs.containsKey(t) && !outputs.containsKey(t) then "larrow"
                    else if !inputs.containsKey(t) && outputs.containsKey(t) then "rarrow"
                    else "ellipse"

                val label =  s"$inStr$t$outStr"
                nodes.put(myNode, s"""$myNode [label="$label", shape=$shape];""")
            }
            case KeyVal(Sym(name), ListTerm(l))
                => {
                    nodes.put(myNode, s"""$myNode [label="${nodeMap.getOrElse(name, name)}", shape=square];""")
                    l.foreach( c => { val c_id = getNodesAndEdges(c, nodes, edges, inputs, outputs); edges.put(myNode, c_id :: edges.getOrElse(myNode, Nil)) })
                }
            case _ 
                => throw new IllegalArgumentException("Not a legal behavior tree node: " + x)
        }
        myNode
    }
}