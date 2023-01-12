package org.aiddl.core.scala.parser

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Real
import org.aiddl.core.scala.representation.Integer
import org.aiddl.core.scala.representation.Rational
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.SetTerm
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.parser.Parser

import java.util.regex.Pattern

import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.representation.EntRef
import org.aiddl.core.scala.representation.FunRef

class ParserSuite extends AnyFunSuite {

    test("Symbol regular expression works") {
        val positive_examples = List("a", "ab", "a.b", "a**A", "=a=", "a.*.B+++", "abc/cde", "-a-1203_aff*", "+", "-", "*", "a?", "-a-", "+a+")
        val negative_examples = List("1", " +a", "?e", "1/3", "10.2", "890")

        //positive_examples.foreach( x => println(x + " -> " + Parser.symRegEx.matches(x)))
        assert( positive_examples.forall( Parser.SymRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.SymRegEx.matches _ ) )
    }

    test("Variable regular expression works") {
        val positive_examples = List("?a", "?ab", "?a.b", "?a**A", "?=a=", "?a.*.B+++", "?abc/cde", "?-a-1203_aff*", "?+", "?-", "?*", "?a?", "?-a-", "?+a+")
        val negative_examples = List("1", " +a", "e", "1/3", "10.2", "890")

        //positive_examples.foreach( x => println(x + " -> " + Parser.varRegEx.matches(x)))
        assert( positive_examples.forall( Parser.VarRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.VarRegEx.matches _ ) )
    }

    test("Integer regular expression works") {
        val positive_examples = List("0", "1", "+1", "-1", "10", "1234567890", "+1230", "-1203")
        val negative_examples = List("01", " 1", "+1e", "-1/3", "10.2", "++890")

        // positive_examples.foreach( x => println(x + " -> " + Parser.intRegEx.matches(x)))
        assert( positive_examples.forall( Parser.IntRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.IntRegEx.matches _ ) )
    }

    test("Binary regular expression works") {
        val positive_examples = List("#b0", "#b1", "#b+1", "#b-1", "#b10", "#b10101110011", "#b+1000", "#b-111")
        val negative_examples = List("#b012", "#b 1", "#b+1e", "#b-1/3", "10.2", "++890", "1", "101", "0")

        //positive_examples.foreach( x => println(x + " -> " + Parser.binRegEx.matches(x)))
        assert( positive_examples.forall( Parser.BinRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.BinRegEx.matches _ ) )
    }

    test("Octagonal regular expression works") {
        val positive_examples = List("#o0", "#o1", "#o+132107", "#o-1", "#o777", "#o016101310711", "#o+1067", "#o-777")
        val negative_examples = List("#o018", "#o 1", "#o+1e", "#o-1/3", "10.2", "++890", "7", "107", "0")

        //positive_examples.foreach( x => println(x + " -> " + Parser.octRegEx.matches(x)))
        assert( positive_examples.forall( Parser.OctRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.OctRegEx.matches _ ) )
    }

    test("Hexadecimal regular expression works") {
        val positive_examples = List("#x0", "#x1", "#x+1A2b0f", "#x-FABCDF", "#xFF00", "#xe16b0cA10d11", "#x+1a67", "#x-fff")
        val negative_examples = List("#x01g", "#x 1", "#x+1l", "#x-1/3", "10.2", "#x++890", "7", "107", "0")

        // positive_examples.foreach( x => println(x + " -> " + Parser.hexRegEx.matches(x)))
        assert( positive_examples.forall( Parser.HexRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.HexRegEx.matches _ ) )
    }

    test("Rational regular expression works") {
        val positive_examples = List("0/1", "1/2", "+10/30", "-1/6", "10/13", "1234567890/22345", "+1230/13", "-1203/174")
        val negative_examples = List("01/2", " 1/4", "+1/e", "- 1/3", "10.2", "++890/23", "1/0")

        //positive_examples.foreach( x => println(x + " -> " + Parser.rationalRegEx.matches(x)))
        assert( positive_examples.forall( Parser.RationalRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.RationalRegEx.matches _ ) )
    }

    test("Real regular expression works") {
        val positive_examples = List("0.1", "1.2", "+10.30", "-1.6", "10.13", "+123.456789022345", "+1230.3", "-1203.174", "3.1415")
        val negative_examples = List("01.2", " 1.4", "+1.e", "- 1.3", "10/2", "++890.23", "1.0 ")

        //positive_examples.foreach( x => println(x + " -> " + Parser.RealRegEx.matches(x)))
        assert( positive_examples.forall( Parser.RealRegEx.matches _ ) )
        assert( !negative_examples.exists( Parser.RealRegEx.matches _ ) )
    }

    test("Parsing single symbolic token") {
        val tokens = List("a")

        assert( List(Sym("a")) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing single variable token") {
        val tokens = List("?X")
        assert( List(Var("X")) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing list of numerical tokens") {
        val tokens = List("42", "-1/2", "3.1415")
        assert( List(Integer(42L), Rational(-1, 2), Real(3.1415)) == Parser.processToken(tokens, Nil, new Container).reverse )
    }

    test("Parsing tuple containing numerical tokens") {
        val tokens = List("(", "42", "-1/2", "3.1415", ")")
        val t = Parser.processToken(tokens, Nil, new Container).head

        t match {
            case Tuple(x1, x2, x3) => {
                assert(x1 == Integer(42)); 
                assert(x2 == Rational(-1, 2)); 
                assert(x3 == Real(3.1415)); 
            }
            case _ => assert(false)
        }

        assert( List(Tuple(Integer(42L), Rational(-1, 2), Real(3.1415))) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing list containing numerical tokens") {
        val tokens = List("[", "42", "-1/2", "3.1415", "]")
        assert( List(ListTerm(Integer(42L), Rational(-1, 2), Real(3.1415))) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing set containing numerical tokens") {
        val tokens = List("{", "42", "-1/2", "3.1415", "}")
        assert( List(SetTerm(Integer(42L), Rational(-1, 2), Real(3.1415))) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing nested tuple and list") {
        val tokens = List("(", "[", "42", "-1/2", "3.1415", "]", ")")
        assert( List(Tuple(ListTerm(Integer(42L), Rational(-1, 2), Real(3.1415)))) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing a key value term") {
        val tokens = List("x", ":", "42")
        assert( List(KeyVal(Sym("x"), Integer(42L))) == Parser.processToken(tokens, Nil, new Container) )
    }

    test("Parsing a module tuple") {
        val tokens = List("(", "#mod", "self", "org.aiddl.eval.test", ")")
        assert( Tuple(Sym("#mod"), Sym("self"), Sym("org.aiddl.eval.test")) == Parser.processToken(tokens, Nil, new Container).head )
    }

    test("Parser loads test aiddl file") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/example-module.aiddl")

        assert( c.resolve(c.getEntry(m, Sym("SR")).get.value) == SetTerm(Sym("d"), Sym("c"), Sym("e")) )
        assert( c.resolve(c.getEntry(m, Sym("D")).get.value) == Num(5) )
        val t = c.resolve(c.getEntry(m, Sym("T")).get.value)
        assert( t == Tuple(KeyVal(Sym("a"), Integer(1L))))

        assert( c.getEntry(m, Sym("K")).get.value == KeyVal(Sym("a"), KeyVal(Sym("b"), Sym("c"))))

        val intFunRef = c.resolve(c.getEntry(m, Sym("IntFunRef")).get.value)
        assert(intFunRef.isInstanceOf[FunRef])

        val extFunRef = c.resolve(c.getEntry(m, Sym("ExtFunRef")).get.value)
        assert(extFunRef.isInstanceOf[FunRef])

        val kvpRef = c.resolve(c.getEntry(m, Sym("KvpRef")).get.value)
        assert(kvpRef.isInstanceOf[KeyVal])
        assert(kvpRef.asKvp.value.isInstanceOf[FunRef])

        val result = SetTerm(Sym("c"), Sym("d"), Sym("e"))
        assert(c.getProcessedValue(m, Sym("SR")) match {
            case Some(value) => {
                value == result
            }
            case _ => false
        })

        assert(c.getProcessedValueOrPanic(m, Sym("SR")) == result)
    }
}