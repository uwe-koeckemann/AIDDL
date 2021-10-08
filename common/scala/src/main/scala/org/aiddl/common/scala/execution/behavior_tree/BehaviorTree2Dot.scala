package org.aiddl.common.scala.execution.behavior_tree

import scala.collection.mutable.HashMap

import java.io.PrintWriter

import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.FunRef
import org.aiddl.core.scala.representation.Str

class BehaviorTree2Dot extends org.aiddl.core.scala.function.Function {

    var nextID = 0;
    val nodeMap = Map( "Sequence" -> "->", "Select" -> "?" )

    def toFile( x: Term, fn: String ) = { val s = this(x); new PrintWriter(fn) { write(s.asStr.value); close } }

    def apply( x: Term ): Term = {
        val v = new HashMap[Int, String]
        val e = new HashMap[Int, List[Int]]
        val sb = new StringBuilder
        this.getNodesAndEdges(x, v, e)

        sb.append("digraph {\n")
        v.foreach( (n, s) => sb.append(s"""\t$s\n"""))
        sb.append("\n")
        e.foreach( (n1, adjList) => { adjList.reverse.foreach( n2 => sb.append(s"""\t$n1 -> $n2\n""") ) })
        sb.append("}")

        Str(sb.toString)
    }

    def getNodesAndEdges( x: Term, nodes: HashMap[Int, String], edges: HashMap[Int, List[Int]] ): Int = {
        val myNode = nextID; 
        this.nextID += 1
        x match {
            case KeyVal(Sym("Leaf"), t)
                => nodes.put(myNode, s"""$myNode [label="$t", shape=ellipse];""")
            case KeyVal(Sym(name), ListTerm(l)) if nodeMap.contains(name)
                => {
                    nodes.put(myNode, s"""$myNode [label="${nodeMap(name)}", shape=square];""")
                    l.foreach( c => { val c_id = getNodesAndEdges(c, nodes, edges); edges.put(myNode, c_id :: edges.getOrElse(myNode, Nil)) })
                }
            case _ 
                => throw new IllegalArgumentException("Not a legal behavior tree node: " + x)
        }
        myNode
    }
}