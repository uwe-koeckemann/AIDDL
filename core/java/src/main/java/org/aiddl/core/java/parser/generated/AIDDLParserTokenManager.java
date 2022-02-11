/* Generated By:JavaCC: Do not edit this line. AIDDLParserTokenManager.java */
package org.aiddl.core.java.parser.generated;
import org.aiddl.core.java.representation.*;

/** Token Manager. */
public class AIDDLParserTokenManager implements AIDDLParserConstants
{

  /** Debug output. */
  public  java.io.PrintStream debugStream = System.out;
  /** Set debug output. */
  public  void setDebugStream(java.io.PrintStream ds) { debugStream = ds; }
private final int jjStopStringLiteralDfa_0(int pos, long active0)
{
   switch (pos)
   {
      default :
         return -1;
   }
}
private final int jjStartNfa_0(int pos, long active0)
{
   return jjMoveNfa_0(jjStopStringLiteralDfa_0(pos, active0), pos + 1);
}
private int jjStopAtPos(int pos, int kind)
{
   jjmatchedKind = kind;
   jjmatchedPos = pos;
   return pos + 1;
}
private int jjMoveStringLiteralDfa0_0()
{
   switch(curChar)
   {
      case 34:
         return jjStartNfaWithStates_0(0, 16, 64);
      case 36:
         return jjStopAtPos(0, 14);
      case 40:
         return jjStopAtPos(0, 7);
      case 41:
         return jjStopAtPos(0, 8);
      case 58:
         return jjStopAtPos(0, 17);
      case 64:
         return jjStopAtPos(0, 13);
      case 91:
         return jjStopAtPos(0, 9);
      case 93:
         return jjStopAtPos(0, 10);
      case 94:
         return jjStopAtPos(0, 15);
      case 95:
         return jjStopAtPos(0, 18);
      case 123:
         return jjStopAtPos(0, 11);
      case 125:
         return jjStopAtPos(0, 12);
      default :
         return jjMoveNfa_0(0, 0);
   }
}
private int jjStartNfaWithStates_0(int pos, int kind, int state)
{
   jjmatchedKind = kind;
   jjmatchedPos = pos;
   try { curChar = input_stream.readChar(); }
   catch(java.io.IOException e) { return pos + 1; }
   return jjMoveNfa_0(state, pos + 1);
}
static final long[] jjbitVec0 = {
   0x0L, 0x0L, 0xffffffffffffffffL, 0xffffffffffffffffL
};
private int jjMoveNfa_0(int startState, int curPos)
{
   int startsAt = 0;
   jjnewStateCnt = 64;
   int i = 1;
   jjstateSet[0] = startState;
   int kind = 0x7fffffff;
   for (;;)
   {
      if (++jjround == 0x7fffffff)
         ReInitRounds();
      if (curChar < 64)
      {
         long l = 1L << curChar;
         do
         {
            switch(jjstateSet[--i])
            {
               case 64:
                  if ((0xfffffffbffffffffL & l) != 0L)
                     jjCheckNAddStates(0, 2);
                  else if (curChar == 34)
                  {
                     if (kind > 30)
                        kind = 30;
                  }
                  break;
               case 0:
                  if ((0x3fe000000000000L & l) != 0L)
                  {
                     if (kind > 23)
                        kind = 23;
                     jjCheckNAddStates(3, 9);
                  }
                  else if ((0x7000a84200000000L & l) != 0L)
                  {
                     if (kind > 28)
                        kind = 28;
                  }
                  else if ((0x40800000000L & l) != 0L)
                  {
                     if (kind > 28)
                        kind = 28;
                     jjCheckNAdd(10);
                  }
                  else if (curChar == 48)
                  {
                     if (kind > 23)
                        kind = 23;
                     jjCheckNAddStates(10, 12);
                  }
                  else if (curChar == 34)
                     jjCheckNAddStates(0, 2);
                  else if (curChar == 59)
                     jjCheckNAddTwoStates(1, 2);
                  else if (curChar == 63)
                     jjstateSet[jjnewStateCnt++] = 18;
                  if ((0x280000000000L & l) != 0L)
                     jjAddStates(13, 21);
                  else if (curChar == 35)
                     jjAddStates(22, 24);
                  else if (curChar == 60)
                     jjCheckNAddTwoStates(26, 15);
                  else if (curChar == 62)
                     jjCheckNAdd(15);
                  else if (curChar == 33)
                     jjCheckNAdd(15);
                  else if (curChar == 61)
                     jjCheckNAdd(13);
                  break;
               case 1:
                  if ((0xfffffffffffffbffL & l) != 0L)
                     jjCheckNAddTwoStates(1, 2);
                  break;
               case 2:
                  if (curChar == 10 && kind > 1)
                     kind = 1;
                  break;
               case 9:
                  if ((0x40800000000L & l) == 0L)
                     break;
                  if (kind > 28)
                     kind = 28;
                  jjCheckNAdd(10);
                  break;
               case 10:
                  if ((0x3ff648000000000L & l) == 0L)
                     break;
                  if (kind > 28)
                     kind = 28;
                  jjCheckNAdd(10);
                  break;
               case 11:
                  if ((0x7000a84200000000L & l) != 0L && kind > 28)
                     kind = 28;
                  break;
               case 12:
               case 26:
                  if (curChar == 61)
                     jjCheckNAdd(13);
                  break;
               case 13:
                  if (curChar == 62 && kind > 28)
                     kind = 28;
                  break;
               case 14:
                  if (curChar == 33)
                     jjCheckNAdd(15);
                  break;
               case 15:
                  if (curChar == 61 && kind > 28)
                     kind = 28;
                  break;
               case 16:
                  if (curChar == 62)
                     jjCheckNAdd(15);
                  break;
               case 17:
                  if (curChar == 63)
                     jjstateSet[jjnewStateCnt++] = 18;
                  break;
               case 19:
                  if ((0x3ff608000000000L & l) == 0L)
                     break;
                  if (kind > 29)
                     kind = 29;
                  jjstateSet[jjnewStateCnt++] = 19;
                  break;
               case 20:
                  if (curChar == 34)
                     jjCheckNAddStates(0, 2);
                  break;
               case 22:
                  jjCheckNAddStates(0, 2);
                  break;
               case 23:
                  if ((0xfffffffbffffffffL & l) != 0L)
                     jjCheckNAddStates(0, 2);
                  break;
               case 24:
                  if (curChar == 34 && kind > 30)
                     kind = 30;
                  break;
               case 25:
                  if (curChar == 60)
                     jjCheckNAddTwoStates(26, 15);
                  break;
               case 27:
                  if ((0x280000000000L & l) != 0L)
                     jjAddStates(13, 21);
                  break;
               case 28:
                  if (curChar == 48)
                     jjCheckNAdd(29);
                  break;
               case 29:
                  if (curChar == 46)
                     jjCheckNAdd(30);
                  break;
               case 30:
                  if ((0x3ff000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(30, 31);
                  break;
               case 32:
                  if ((0x280000000000L & l) != 0L)
                     jjAddStates(25, 26);
                  break;
               case 33:
                  if (curChar == 48 && kind > 19)
                     kind = 19;
                  break;
               case 34:
                  if ((0x3fe000000000000L & l) == 0L)
                     break;
                  if (kind > 19)
                     kind = 19;
                  jjCheckNAdd(35);
                  break;
               case 35:
                  if ((0x3ff000000000000L & l) == 0L)
                     break;
                  if (kind > 19)
                     kind = 19;
                  jjCheckNAdd(35);
                  break;
               case 36:
                  if ((0x3fe000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(37, 29);
                  break;
               case 37:
                  if ((0x3ff000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(37, 29);
                  break;
               case 38:
                  if (curChar == 48 && kind > 23)
                     kind = 23;
                  break;
               case 39:
                  if ((0x3fe000000000000L & l) == 0L)
                     break;
                  if (kind > 23)
                     kind = 23;
                  jjCheckNAdd(40);
                  break;
               case 40:
                  if ((0x3ff000000000000L & l) == 0L)
                     break;
                  if (kind > 23)
                     kind = 23;
                  jjCheckNAdd(40);
                  break;
               case 41:
                  if (curChar == 48)
                     jjCheckNAdd(42);
                  break;
               case 42:
                  if (curChar == 47)
                     jjstateSet[jjnewStateCnt++] = 43;
                  break;
               case 43:
                  if ((0x3fe000000000000L & l) == 0L)
                     break;
                  if (kind > 24)
                     kind = 24;
                  jjCheckNAdd(44);
                  break;
               case 44:
                  if ((0x3ff000000000000L & l) == 0L)
                     break;
                  if (kind > 24)
                     kind = 24;
                  jjCheckNAdd(44);
                  break;
               case 45:
                  if ((0x3fe000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(46, 42);
                  break;
               case 46:
                  if ((0x3ff000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(46, 42);
                  break;
               case 47:
                  if (curChar == 48)
                     jjCheckNAdd(48);
                  break;
               case 48:
                  if (curChar == 46)
                     jjCheckNAdd(49);
                  break;
               case 49:
                  if ((0x3ff000000000000L & l) == 0L)
                     break;
                  if (kind > 25)
                     kind = 25;
                  jjCheckNAdd(49);
                  break;
               case 50:
                  if ((0x3fe000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(51, 48);
                  break;
               case 51:
                  if ((0x3ff000000000000L & l) != 0L)
                     jjCheckNAddTwoStates(51, 48);
                  break;
               case 52:
                  if (curChar != 48)
                     break;
                  if (kind > 23)
                     kind = 23;
                  jjCheckNAddStates(10, 12);
                  break;
               case 53:
                  if ((0x3fe000000000000L & l) == 0L)
                     break;
                  if (kind > 23)
                     kind = 23;
                  jjCheckNAddStates(3, 9);
                  break;
               case 54:
                  if (curChar == 35)
                     jjAddStates(22, 24);
                  break;
               case 56:
                  if ((0x280000000000L & l) != 0L)
                     jjCheckNAdd(57);
                  break;
               case 57:
                  if ((0x3000000000000L & l) == 0L)
                     break;
                  if (kind > 20)
                     kind = 20;
                  jjCheckNAdd(57);
                  break;
               case 59:
                  if ((0x280000000000L & l) != 0L)
                     jjCheckNAdd(60);
                  break;
               case 60:
                  if ((0x3ff000000000000L & l) == 0L)
                     break;
                  if (kind > 21)
                     kind = 21;
                  jjCheckNAdd(60);
                  break;
               case 62:
                  if ((0x280000000000L & l) != 0L)
                     jjCheckNAdd(63);
                  break;
               case 63:
                  if ((0xff000000000000L & l) == 0L)
                     break;
                  if (kind > 22)
                     kind = 22;
                  jjCheckNAdd(63);
                  break;
               default : break;
            }
         } while(i != startsAt);
      }
      else if (curChar < 128)
      {
         long l = 1L << (curChar & 077);
         do
         {
            switch(jjstateSet[--i])
            {
               case 64:
                  if ((0xffffffffefffffffL & l) != 0L)
                     jjCheckNAddStates(0, 2);
                  else if (curChar == 92)
                     jjstateSet[jjnewStateCnt++] = 22;
                  break;
               case 0:
                  if ((0x7fffffe07fffffeL & l) != 0L)
                  {
                     if (kind > 28)
                        kind = 28;
                     jjCheckNAdd(10);
                  }
                  else if (curChar == 124)
                  {
                     if (kind > 28)
                        kind = 28;
                  }
                  if (curChar == 78)
                     jjstateSet[jjnewStateCnt++] = 7;
                  else if (curChar == 73)
                     jjstateSet[jjnewStateCnt++] = 4;
                  break;
               case 1:
                  jjAddStates(27, 28);
                  break;
               case 3:
                  if (curChar == 73)
                     jjstateSet[jjnewStateCnt++] = 4;
                  break;
               case 4:
                  if (curChar == 78)
                     jjstateSet[jjnewStateCnt++] = 5;
                  break;
               case 5:
                  if (curChar == 70 && kind > 26)
                     kind = 26;
                  break;
               case 6:
                  if (curChar == 78)
                     jjstateSet[jjnewStateCnt++] = 7;
                  break;
               case 7:
                  if (curChar == 97)
                     jjstateSet[jjnewStateCnt++] = 8;
                  break;
               case 8:
                  if (curChar == 78 && kind > 27)
                     kind = 27;
                  break;
               case 9:
                  if ((0x7fffffe07fffffeL & l) == 0L)
                     break;
                  if (kind > 28)
                     kind = 28;
                  jjCheckNAdd(10);
                  break;
               case 10:
                  if ((0x7fffffe87fffffeL & l) == 0L)
                     break;
                  if (kind > 28)
                     kind = 28;
                  jjCheckNAdd(10);
                  break;
               case 11:
                  if (curChar == 124 && kind > 28)
                     kind = 28;
                  break;
               case 18:
                  if ((0x7fffffe07fffffeL & l) == 0L)
                     break;
                  if (kind > 29)
                     kind = 29;
                  jjCheckNAdd(19);
                  break;
               case 19:
                  if ((0x7fffffe87fffffeL & l) == 0L)
                     break;
                  if (kind > 29)
                     kind = 29;
                  jjCheckNAdd(19);
                  break;
               case 21:
                  if (curChar == 92)
                     jjstateSet[jjnewStateCnt++] = 22;
                  break;
               case 22:
                  jjCheckNAddStates(0, 2);
                  break;
               case 23:
                  if ((0xffffffffefffffffL & l) != 0L)
                     jjCheckNAddStates(0, 2);
                  break;
               case 31:
                  if ((0x2000000020L & l) != 0L)
                     jjAddStates(29, 31);
                  break;
               case 55:
                  if (curChar == 98)
                     jjAddStates(32, 33);
                  break;
               case 58:
                  if (curChar == 120)
                     jjCheckNAddTwoStates(59, 60);
                  break;
               case 60:
                  if ((0x7e0000007eL & l) == 0L)
                     break;
                  if (kind > 21)
                     kind = 21;
                  jjCheckNAdd(60);
                  break;
               case 61:
                  if (curChar == 111)
                     jjAddStates(34, 35);
                  break;
               default : break;
            }
         } while(i != startsAt);
      }
      else
      {
         int i2 = (curChar & 0xff) >> 6;
         long l2 = 1L << (curChar & 077);
         do
         {
            switch(jjstateSet[--i])
            {
               case 64:
               case 23:
               case 22:
                  if ((jjbitVec0[i2] & l2) != 0L)
                     jjCheckNAddStates(0, 2);
                  break;
               case 1:
                  if ((jjbitVec0[i2] & l2) != 0L)
                     jjAddStates(27, 28);
                  break;
               default : break;
            }
         } while(i != startsAt);
      }
      if (kind != 0x7fffffff)
      {
         jjmatchedKind = kind;
         jjmatchedPos = curPos;
         kind = 0x7fffffff;
      }
      ++curPos;
      if ((i = jjnewStateCnt) == (startsAt = 64 - (jjnewStateCnt = startsAt)))
         return curPos;
      try { curChar = input_stream.readChar(); }
      catch(java.io.IOException e) { return curPos; }
   }
}
static final int[] jjnextStates = {
   21, 23, 24, 37, 29, 40, 46, 42, 51, 48, 29, 42, 48, 28, 36, 38, 
   39, 41, 45, 47, 50, 3, 55, 58, 61, 33, 34, 1, 2, 32, 33, 34, 
   56, 57, 62, 63, 
};

/** Token literal values. */
public static final String[] jjstrLiteralImages = {
"", null, null, null, null, null, null, "\50", "\51", "\133", "\135", "\173", 
"\175", "\100", "\44", "\136", "\42", "\72", "\137", null, null, null, null, null, 
null, null, null, null, null, null, null, };

/** Lexer state names. */
public static final String[] lexStateNames = {
   "DEFAULT",
};
static final long[] jjtoToken = {
   0x7fffff81L, 
};
static final long[] jjtoSkip = {
   0x7eL, 
};
protected SimpleCharStream input_stream;
private final int[] jjrounds = new int[64];
private final int[] jjstateSet = new int[128];
protected char curChar;
/** Constructor. */
public AIDDLParserTokenManager(SimpleCharStream stream){
   if (SimpleCharStream.staticFlag)
      throw new Error("ERROR: Cannot use a static CharStream class with a non-static lexical analyzer.");
   input_stream = stream;
}

/** Constructor. */
public AIDDLParserTokenManager(SimpleCharStream stream, int lexState){
   this(stream);
   SwitchTo(lexState);
}

/** Reinitialise parser. */
public void ReInit(SimpleCharStream stream)
{
   jjmatchedPos = jjnewStateCnt = 0;
   curLexState = defaultLexState;
   input_stream = stream;
   ReInitRounds();
}
private void ReInitRounds()
{
   int i;
   jjround = 0x80000001;
   for (i = 64; i-- > 0;)
      jjrounds[i] = 0x80000000;
}

/** Reinitialise parser. */
public void ReInit(SimpleCharStream stream, int lexState)
{
   ReInit(stream);
   SwitchTo(lexState);
}

/** Switch to specified lex state. */
public void SwitchTo(int lexState)
{
   if (lexState >= 1 || lexState < 0)
      throw new TokenMgrError("Error: Ignoring invalid lexical state : " + lexState + ". State unchanged.", TokenMgrError.INVALID_LEXICAL_STATE);
   else
      curLexState = lexState;
}

protected Token jjFillToken()
{
   final Token t;
   final String curTokenImage;
   final int beginLine;
   final int endLine;
   final int beginColumn;
   final int endColumn;
   String im = jjstrLiteralImages[jjmatchedKind];
   curTokenImage = (im == null) ? input_stream.GetImage() : im;
   beginLine = input_stream.getBeginLine();
   beginColumn = input_stream.getBeginColumn();
   endLine = input_stream.getEndLine();
   endColumn = input_stream.getEndColumn();
   t = Token.newToken(jjmatchedKind, curTokenImage);

   t.beginLine = beginLine;
   t.endLine = endLine;
   t.beginColumn = beginColumn;
   t.endColumn = endColumn;

   return t;
}

int curLexState = 0;
int defaultLexState = 0;
int jjnewStateCnt;
int jjround;
int jjmatchedPos;
int jjmatchedKind;

/** Get the next Token. */
public Token getNextToken() 
{
  Token matchedToken;
  int curPos = 0;

  EOFLoop :
  for (;;)
  {
   try
   {
      curChar = input_stream.BeginToken();
   }
   catch(java.io.IOException e)
   {
      jjmatchedKind = 0;
      matchedToken = jjFillToken();
      return matchedToken;
   }

   try { input_stream.backup(0);
      while (curChar <= 44 && (0x100100002600L & (1L << curChar)) != 0L)
         curChar = input_stream.BeginToken();
   }
   catch (java.io.IOException e1) { continue EOFLoop; }
   jjmatchedKind = 0x7fffffff;
   jjmatchedPos = 0;
   curPos = jjMoveStringLiteralDfa0_0();
   if (jjmatchedKind != 0x7fffffff)
   {
      if (jjmatchedPos + 1 < curPos)
         input_stream.backup(curPos - jjmatchedPos - 1);
      if ((jjtoToken[jjmatchedKind >> 6] & (1L << (jjmatchedKind & 077))) != 0L)
      {
         matchedToken = jjFillToken();
         return matchedToken;
      }
      else
      {
         continue EOFLoop;
      }
   }
   int error_line = input_stream.getEndLine();
   int error_column = input_stream.getEndColumn();
   String error_after = null;
   boolean EOFSeen = false;
   try { input_stream.readChar(); input_stream.backup(1); }
   catch (java.io.IOException e1) {
      EOFSeen = true;
      error_after = curPos <= 1 ? "" : input_stream.GetImage();
      if (curChar == '\n' || curChar == '\r') {
         error_line++;
         error_column = 0;
      }
      else
         error_column++;
   }
   if (!EOFSeen) {
      input_stream.backup(1);
      error_after = curPos <= 1 ? "" : input_stream.GetImage();
   }
   throw new TokenMgrError(EOFSeen, curLexState, error_line, error_column, error_after, curChar, TokenMgrError.LEXICAL_ERROR);
  }
}

private void jjCheckNAdd(int state)
{
   if (jjrounds[state] != jjround)
   {
      jjstateSet[jjnewStateCnt++] = state;
      jjrounds[state] = jjround;
   }
}
private void jjAddStates(int start, int end)
{
   do {
      jjstateSet[jjnewStateCnt++] = jjnextStates[start];
   } while (start++ != end);
}
private void jjCheckNAddTwoStates(int state1, int state2)
{
   jjCheckNAdd(state1);
   jjCheckNAdd(state2);
}

private void jjCheckNAddStates(int start, int end)
{
   do {
      jjCheckNAdd(jjnextStates[start]);
   } while (start++ != end);
}

}