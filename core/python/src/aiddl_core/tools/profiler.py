import traceback


class Profiler:
    def probe(n):
        try:
            1 / 0
        except ZeroDivisionError:
            error = traceback.format_exc()
            exit()
        #print(error)
        #print(type(error))
