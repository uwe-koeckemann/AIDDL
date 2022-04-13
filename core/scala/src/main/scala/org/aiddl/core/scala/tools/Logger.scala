package org.aiddl.core.scala.tools

import org.aiddl.core.scala.representation._

object Logger {
    var depth = 0
    var incStr = "| "

    def ++ = depth += 1
    def -- = {
        depth -= 1
        if (depth < 0) throw new IllegalStateException("Logger depth decreased below zero.")
    }

    def msg(name: String, v: Int)(l: Int, msg: => String) = 
      if ( l <= v ) { println(incStr * depth + "[" + name + "] " + msg) }

    def msgInc(name: String, v: Int)(l: Int, msg: => String) = {
      if ( l <= v ) { println(incStr * depth + "[" + name + "] " + msg); depth += 1; }
    }

    def msgDec(name: String, v: Int)(l: Int, msg: => String) = {
      if ( l <= v ) { depth -= 1; println(incStr * depth + "[" + name + "] " + msg); }
    }

    def simpleTabbing( n: Int ): String = "  " * n

    def prettyPrint( t: Term, depth: Int ): String = {
        val base = t.toString
        if ( base.length <= 80 ) simpleTabbing(depth) + base
        else {
            val sB = new StringBuilder
            t match {
                case s: SetTerm => {
                    sB.append(simpleTabbing(depth))
                    sB.append("{\n")
                    s.foreach( e => {sB.append(prettyPrint(e, depth+1)); sB.append("\n")})
                    sB.append(simpleTabbing(depth))
                    sB.append("}\n")
                }
                case s: ListTerm => {
                    sB.append(simpleTabbing(depth))
                    sB.append("[\n")
                    s.foreach( e => {sB.append(prettyPrint(e, depth+1)); sB.append("\n")})
                    sB.append(simpleTabbing(depth))
                    sB.append("]\n")
                }
                case s: Tuple => {
                    sB.append(simpleTabbing(depth))
                    sB.append("(\n")
                    s.foreach( e => {sB.append(prettyPrint(e, depth+1)); sB.append("\n")})
                    sB.append(simpleTabbing(depth))
                    sB.append(")\n")
                }
                case s: KeyVal => {
                    sB.append(prettyPrint(s.key, depth))
                    sB.append(":\n")
                    sB.append(prettyPrint(s.value, depth+1))
                }
                case _=> {
                    sB.append(simpleTabbing(depth))
                    sB.append(base)
                }
            }
            sB.toString
        }
    }
}
