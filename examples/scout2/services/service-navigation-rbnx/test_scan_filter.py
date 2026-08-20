import math
import unittest

from nav2_wrapper.scan_filter_core import filter_ranges


class ScanFilterTest(unittest.TestCase):
    def test_single_ghost_point_is_removed(self):
        values = [math.inf] * 9
        values[4] = 1.0
        filtered, removed = filter_ranges(values, 0.5, 12.0)
        self.assertEqual(removed, [4])
        self.assertTrue(math.isinf(filtered[4]))

    def test_three_bin_small_obstacle_is_retained(self):
        values = [math.inf] * 9
        values[3:6] = [1.02, 1.0, 1.03]
        filtered, removed = filter_ranges(values, 0.5, 12.0)
        self.assertEqual(removed, [])
        self.assertEqual(filtered[3:6], values[3:6])

    def test_two_bin_speckle_cluster_is_removed(self):
        values = [math.inf] * 9
        values[4:6] = [0.95, 0.98]
        filtered, removed = filter_ranges(values, 0.5, 12.0)
        self.assertEqual(removed, [4, 5])
        self.assertTrue(all(math.isinf(filtered[i]) for i in removed))

    def test_nearby_bins_at_different_ranges_do_not_support_each_other(self):
        values = [math.inf] * 9
        values[3:6] = [1.0, 1.5, 2.0]
        _, removed = filter_ranges(values, 0.5, 12.0)
        self.assertEqual(removed, [3, 4, 5])


if __name__ == "__main__":
    unittest.main()
