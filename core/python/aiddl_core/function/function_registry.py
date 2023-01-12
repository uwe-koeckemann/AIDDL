from aiddl_core.representation.sym import Sym
from aiddl_core.representation.var import Var
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.list import List

from aiddl_core.function.eval.type import TypeCheckFunction, GenericTypeConstructor

from aiddl_core.function.function import InterfaceImplementationMixin, NamedFunction, LambdaFunction
import aiddl_core.function as fun_uri

DEF = Sym("#def")


class FunctionRegistry:
    def __init__(self):
        self.functions = {}
        self.interfaces = {}
        self.interface_implementations = {}

    def add_function(self, name, f):
        self.functions[name] = f
        for k in self.interface_implementations.keys():
            self.interface_implementations[k] = filter(
                name.__ne__,
                self.interface_implementations[k])
        if isinstance(f, InterfaceImplementationMixin):
            uri = f.get_interface_uri()
            if uri not in self.interface_implementations.keys():
                self.interface_implementations[uri] = []
            self.interface_implementations[uri].append(name)

    def get_function(self, name):
        if isinstance(name, Tuple) \
           and len(name) == 3 \
           and name[0] == fun_uri.LAMBDA:
            return self.lambda_factory(name)
        if name in self.functions.keys():
            return self.functions[name]
        return None

    def get_function_or_panic(self, name):
        if isinstance(name, Tuple) \
           and len(name) == 3 \
           and name[0] == fun_uri.LAMBDA:
            return self.lambda_factory(name)
        if name in self.functions.keys():
            return self.functions[name]
        raise ValueError("Function not registered:", str(name))

    def get_function_or_default(self, name, f_def):
        if isinstance(name, Tuple) \
           and len(name) == 3 \
           and name[0] == fun_uri.LAMBDA:
            return self.lambda_factory(name)
        if name in self.functions.keys():
            return self.functions[name]
        raise ValueError(f_def)

    def get_interface_implementations(self, uri):
        if uri in self.interface_implementations:
            return List(self.interface_implementations[uri])
        return List()

    def lambda_factory(self, n):
        x = n.get(1)
        f = n.get(2)
        e = self.functions[fun_uri.EVAL]
        return LambdaFunction(x, f, e)

    def has_function(self, name):
        return name in self.functions.keys()

    def get_registered_names(self):
        return list(self.functions.keys())

    def load_container_interfaces(self, C):
        evaluator = self.get_function(fun_uri.EVAL)
        for m in C.get_module_names():
            for e in C.get_matching_entries(m, Sym("#interface"), Var()):
                uri = evaluator(e.value[Sym("uri")])
                interface_term = evaluator(e.value)
                self.interfaces[uri] = interface_term

    # def get_function_list(self, m, C):
    #     L = []
    #     for e in C.get_matching_entries(m, Symbolic("#functions"), Variable()):
    #         for t in e.get_value():
    #             L.append(t)
    #     return List(L)
    #
    # def load_req_python_functions(self, C):
    #     for m in C.get_module_names():
    #         functions = self.get_function_list(m, C) # C.get_entry(Symbolic("functions"), module=m)
    #         # print("Functions:", functions)
    #         if len(functions) > 0:
    #             missing = False
    #             for f in functions:
    #                 if not self.has_function(f[0]):
    #                     missing = True
    #                     break
    #             if missing:
    #                 loader_mod = m + Symbolic("python")
    #                 if parser.is_known_module(loader_mod):
    #                     lu = parser.get_mod_file_lookup(parser.collect_aiddl_paths([]))
    #
    #                     evalutaor = self.get_function(EVAL)
    #                     parser.parse_internal(lu[loader_mod], C, self, ".")
    #
    #                     for e in C.get_matching_entries(loader_mod, Symbolic("#on-load"), Variable()):
    #                         load = e.get_value()
    #
    #                         if isinstance(load, Tuple):
    #                             evalutaor(load)
    #                         else:
    #                             for call in load:
    #                                 evalutaor(call)
    #
    #                     # load_request = C.get_entry(Symbolic("load"), module=loader_mod)
    #                     # if load_request is not None:
    #                     #     rHandler = RequestHandler(C, self)
    #                     #     # rHandler.verbose = True
    #                     #     rHandler.satisfy_request(load_request.get_value(), Symbolic("NIL"))
    #                 # else:
    #                 #     print("Could not find loader module:", loader_mod)
    #                 for f in functions:
    #                     if not self.has_function(f[0]):
    #                         print("[Warning]", f[0], ": Missing python implementation")

    def load_type_functions(self, C):
        evaluator = self.get_function(fun_uri.EVAL)
        for m in C.get_module_names():
            for e in C.get_matching_entries(m, Sym("#type"), Var()):
                if isinstance(e.name, Sym):
                    uri = m + e.name
                    evaluator.set_follow_references(True)
                    #evaluator.set_verbose(True)
                    type_def = evaluator(e.value)
                    evaluator.set_follow_references(False)
                    #evaluator.set_verbose(False)
                    type_fun = TypeCheckFunction(type_def, evaluator)
                    self.add_function(uri, type_fun)

                    interface_term = evaluator(e.value)
                    self.interfaces[uri] = interface_term
                    # print("Loaded type:", uri, "with def", type_def)
                elif isinstance(e.name, Tuple):
                    base_uri = m + e.name[0]
                    evaluator.set_follow_references(True)
                    type_def = evaluator(e.value)
                    evaluator.set_follow_references(False)
                    arg_list = []
                    for i in range(1, len(e.name)):
                        arg_list.append(e.name[i])
                    if len(arg_list) == 1:
                        gen_args = arg_list[0]
                    else:
                        gen_args = Tuple(arg_list)
                    type_fun = GenericTypeConstructor(base_uri, gen_args, type_def, evaluator, self)
                    self.add_function(base_uri, type_fun)


    def load_def(self, C):
        for m in C.get_module_names():
            for e in C.get_matching_entries(m, DEF, Var()):
                if isinstance(e.name, Sym):
                    uri = m + e.name
                    f = NamedFunction(uri,
                                      e.value,
                                      self.functions[fun_uri.EVAL])
                    # print("Loading:", uri)
                else:
                    uri = m + e.name[0]

                    args = None
                    if len(e.name) == 2:
                        args = e.name[1]
                    else:
                        arg_list = []
                        for i in range(1, len(e.name)):
                            arg_list.append(e.name[i])
                        args = Tuple(arg_list)
                    # print("Loading:", uri)
                    f = NamedFunction(uri,
                                      e.value,
                                      self.functions[fun_uri.EVAL],
                                      args=args)
                self.add_function(uri, f)
