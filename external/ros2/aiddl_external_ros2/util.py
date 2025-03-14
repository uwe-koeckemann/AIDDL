
def load_class_from_string(uri):
    mods = ".".join(uri.split(".")[:-1])
    class_str = uri.split(".")[-1]
    mod = __import__(mods, fromlist=[class_str])
    return getattr(mod, class_str)
