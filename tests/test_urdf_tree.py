import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Fusion_URDF_Exporter_ROS2.core.urdf_tree import make_urdf_tree


def test_make_urdf_tree_drops_cycles_and_duplicate_parents():
    joints = {
        'root_to_a': {'parent': 'base_link', 'child': 'a'},
        'a_to_b': {'parent': 'a', 'child': 'b'},
        'b_to_a': {'parent': 'b', 'child': 'a'},
        'root_to_b': {'parent': 'base_link', 'child': 'b'},
    }

    kept, warnings = make_urdf_tree(joints)

    assert list(kept) == ['root_to_a', 'a_to_b']
    assert len(warnings) == 2
    assert any('b_to_a' in warning for warning in warnings)
    assert any('root_to_b' in warning for warning in warnings)


def test_make_urdf_tree_drops_disconnected_joints():
    joints = {
        'root_to_a': {'parent': 'base_link', 'child': 'a'},
        'ghost_to_b': {'parent': 'ghost', 'child': 'b'},
    }

    kept, warnings = make_urdf_tree(joints)

    assert list(kept) == ['root_to_a']
    assert warnings == [
        'ghost_to_b (ghost -> b) dropped: not reachable from base_link or closes a loop'
    ]
