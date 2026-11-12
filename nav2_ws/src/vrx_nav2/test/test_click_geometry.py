import math
import pytest
from vrx_nav2.click_geometry import world_to_map, aligned_world_to_map


def test_sim_spawn_is_map_origin():
    assert world_to_map(-532., 162.) == (0., 0.)


@pytest.mark.parametrize('x,y', [(4., 0.), (7., 3.), (-2., -5.)])
def test_map_goal_roundtrip(x, y):
    c, s = math.cos(1.), math.sin(1.)
    assert world_to_map(-532+c*x-s*y, 162+s*x+c*y) == pytest.approx((x, y))


def test_invalid_world_click():
    with pytest.raises(ValueError):
        world_to_map(math.nan, 0.)


def test_click_alignment_with_shifted_gps_datum():
    # World east corresponds to map north; the boat is already away from origin.
    assert aligned_world_to_map(12., 20., (10., 20., 0.), (3., 4., math.pi/2)) == pytest.approx((3., 6.))
    assert aligned_world_to_map(10., 20., (10., 20., 0.), (3., 4., math.pi/2)) == pytest.approx((3., 4.))
