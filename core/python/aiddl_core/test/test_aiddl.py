import unittest
import os
import aiddl_core.unit_test.unit_test_runner as utest

FILE_PATH = os.path.dirname(os.path.abspath(__file__))


class TestAiddl(unittest.TestCase):

    def test_core(self):
        test_file = os.path.join(FILE_PATH, "aiddl", "test.aiddl")
        # resource_path = "aiddl/" # '/'.join(('aiddl'))
        # test_file = resource_path + "/test.aiddl"
        utest.run_aiddl_test_file(test_file)

    def test_types(self):
        test_file = os.path.join(FILE_PATH, "aiddl", "test-types.aiddl")
        # resource_path = "aiddl/" # "'/'.join(('aiddl'))
        # test_file = resource_path + "/test-types.aiddl"
        utest.run_aiddl_test_file(test_file)
