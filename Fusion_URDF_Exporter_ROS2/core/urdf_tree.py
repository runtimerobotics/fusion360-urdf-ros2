from collections import OrderedDict


def make_urdf_tree(joints_dict, root_link='base_link'):
    """
    Return a URDF-compatible spanning tree and warnings for dropped joints.

    Fusion assemblies can contain closed loops or several joints targeting the
    same child occurrence. URDF cannot represent those graphs: each child link
    must have exactly one parent and the model must be a tree rooted at
    ``base_link``. This function keeps the first reachable parent-child edge
    for each child and drops loop-closing or disconnected edges.
    """
    kept = OrderedDict()
    dropped = []
    known_links = {root_link}
    pending = OrderedDict(joints_dict)

    while pending:
        progressed = False
        for joint_name, joint_data in list(pending.items()):
            parent = joint_data['parent']
            child = joint_data['child']

            if child == root_link:
                dropped.append(
                    (joint_name, parent, child, 'child is the root link')
                )
                del pending[joint_name]
                progressed = True
                continue

            if child in known_links:
                dropped.append(
                    (joint_name, parent, child, 'child already has a parent')
                )
                del pending[joint_name]
                progressed = True
                continue

            if parent in known_links:
                kept[joint_name] = joint_data
                known_links.add(child)
                del pending[joint_name]
                progressed = True

        if not progressed:
            for joint_name, joint_data in pending.items():
                dropped.append((
                    joint_name,
                    joint_data['parent'],
                    joint_data['child'],
                    'not reachable from base_link or closes a loop',
                ))
            break

    warnings = [
        '{} ({} -> {}) dropped: {}'.format(joint, parent, child, reason)
        for joint, parent, child, reason in dropped
    ]
    return kept, warnings
