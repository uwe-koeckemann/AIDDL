import unittest

from aiddl_core.container import Container
from aiddl_core.parser import Parser
from aiddl_core.representation import KeyVal


class TestParser(unittest.TestCase):
    def test_parse_key_value_string(self):
        parser = Parser(Container())

        kvp = parser.parse_term("k:v")
        print(kvp)
        assert isinstance(kvp, KeyVal)