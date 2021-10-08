class CallRequestFunction:
    def __init__(self, call, req_handler):
        self.C = call
        self.req_handler = req_handler

    def __call__(self, x):
        request = x[0]
        module = x[1]
        return_entry_name = x[2]

        self.C.add_module(module)
        self.req_handler.satisfy_request(request, module)

        r = self.C.get_entry(return_entry_name, module=module).get_value()
        return r