from aiddl_core.container import Container
from aiddl_core.representation import Sym
from aiddl_core.representation import Var
from aiddl_core.representation import Str
from aiddl_core.representation import Int
from aiddl_core.representation import Real
from aiddl_core.representation import Rat
from aiddl_core.representation import Boolean
from aiddl_core.representation import Inf
from aiddl_core.representation import NaN
from aiddl_core.representation import Set
from aiddl_core.representation import List
from aiddl_core.representation import Tuple
from aiddl_core.representation import KeyVal
from aiddl_core.representation import EntRef
from aiddl_core.representation import FunRef

from aiddl_external_grpc_python.generated.aiddl_pb2 import Term as PbTerm
from aiddl_external_grpc_python.generated.aiddl_pb2 import CollectionTerm as PbCollection
from aiddl_external_grpc_python.generated.aiddl_pb2 import NAN
from aiddl_external_grpc_python.generated.aiddl_pb2 import INF_POS
from aiddl_external_grpc_python.generated.aiddl_pb2 import INF_NEG
from aiddl_external_grpc_python.generated.aiddl_pb2 import SET
from aiddl_external_grpc_python.generated.aiddl_pb2 import LIST
from aiddl_external_grpc_python.generated.aiddl_pb2 import TUPLE


class Converter:
    def __init__(self, container: Container):
        self.container = container

    def aiddl2pb(self, x):
        r = PbTerm()
        if isinstance(x, Boolean):
            r.boolean = x.unpack()
        elif isinstance(x, Sym):
            r.sym = x.string
        elif isinstance(x, Var):
            if str(x)[0] == "_":
                r.var = "_"
            else:
                r.var = str(x)[1:]
        elif isinstance(x, Str):
            r.str = x.string

        elif isinstance(x, Int):
            r.int = x.int_value
        elif isinstance(x, Real):
            r.real = x.real_value
        elif isinstance(x, Rat):
            r.rational.nominator = x.nominator
            r.rational.denominator = x.denominator
        elif isinstance(x, Inf):
            if x.is_inf_pos():
                r.other_numerical = INF_POS
            elif x.is_inf_neg():
                r.other_numerical = INF_NEG
        elif isinstance(x, NaN):
            r.other_numerical = NAN
        elif isinstance(x, Set):
            r.collection.colType = SET
            for e in x:
                r.collection.data.append(self.aiddl2pb(e))
        elif isinstance(x, List):
            r.collection.colType = LIST
            for e in x:
                r.collection.data.append(self.aiddl2pb(e))
        elif isinstance(x, Tuple):
            r.collection.colType = TUPLE
            for e in x:
                r.collection.data.append(self.aiddl2pb(e))
        elif isinstance(x, KeyVal):
            r.kvp.key.CopyFrom( self.aiddl2pb(x.key))
            r.kvp.value.CopyFrom(self.aiddl2pb(x.value))
        elif isinstance(x, EntRef):
            r.ent_ref.module.CopyFrom(self.aiddl2pb(x.module))
            r.ent_ref.name.CopyFrom(self.aiddl2pb(x.target))
            r.ent_ref.alias.CopyFrom(self.aiddl2pb(x.alias))
        elif isinstance(x, FunRef):
            r.fun_ref = str(x.function_uri)
        return r

    def pb2aiddl(self, x):
        term_type = x.WhichOneof('term_type')
        if term_type == 'sym':
            return Sym(x.sym)
        elif term_type == 'var':
            if x.var == '_':
                return Var()
            else:
                return Var(x.var)
        elif term_type == 'str':
            return Str(x.str)
        elif term_type == 'boolean':
            return Boolean(x.boolean)
        elif term_type == 'int':
            return Int(x.int)
        elif term_type == 'real':
            return Real(x.real)
        elif term_type == 'rational':
            return Rat(x.rational.nominator, x.rational.denominator)
        elif term_type == 'other_numerical':
            if x.other_numerical == INF_POS:
                return Inf.pos()
            elif x.other_numerical == INF_NEG:
                return Inf.neg()
            elif x.other_numerical == NAN:
                return NaN()
        elif term_type == 'collection':
            if x.collection.colType == SET:
                return Set([self.pb2aiddl(e) for e in x.collection.data])
            elif x.collection.colType == LIST:
                return List([self.pb2aiddl(e) for e in x.collection.data])
            elif x.collection.colType == TUPLE:
                return Tuple([self.pb2aiddl(e) for e in x.collection.data])
        elif term_type == 'kvp':
            return KeyVal(self.pb2aiddl(x.kvp.key),
                          self.pb2aiddl(x.kvp.value))
        elif term_type == 'ent_ref':
            return EntRef(
                self.pb2aiddl(x.ent_ref.name),
                self.pb2aiddl(x.ent_ref.module),
                self.pb2aiddl(x.ent_ref.alias))
        elif term_type == 'fun_ref':
            return FunRef(Sym(x.fun_ref), c.fun_reg)


c = Container()
conv = Converter(c)

x = Sym("x")
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Var("x")
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Var()
assert conv.pb2aiddl(conv.aiddl2pb(x)).is_anonymous
x = Str("This is a string!")
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Boolean(True)
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Int(35)
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Real(35.2345)
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Rat(1, 4)
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Inf.pos()
assert conv.pb2aiddl(conv.aiddl2pb(x)).is_inf_pos
x = Inf.neg()
assert conv.pb2aiddl(conv.aiddl2pb(x)).is_inf_neg
x = NaN()
assert conv.pb2aiddl(conv.aiddl2pb(x)).is_nan
x = Set(Int(1), Sym("a"), Rat(1, 4))
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = List(Int(1), Sym("a"), Rat(1, 4))
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = Tuple(Int(1), Sym("a"), Rat(1, 4))
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = KeyVal(Sym("x"), Int(4))
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = EntRef(Sym("m"), Tuple(Sym("x"), Int(1)), Sym("alias"))
assert x == conv.pb2aiddl(conv.aiddl2pb(x))
x = FunRef(Sym("f"), c.fun_reg)
assert x == conv.pb2aiddl(conv.aiddl2pb(x))

