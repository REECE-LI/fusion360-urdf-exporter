# -*- coding: utf-8 -*-
"""Build and serialize the top-level Fusion joint tree."""

import math
import re
from xml.etree.ElementTree import Element, SubElement

import adsk
import adsk.fusion

from ..utils import utils


VERSION_SUFFIX = re.compile(r'\s+v\d+:\d+$', re.IGNORECASE)
UNSAFE_LINK_CHARACTERS = re.compile(r'[\s:/\\()]+')


def logical_name(name):
    """Remove Fusion's trailing version and occurrence suffix."""
    return VERSION_SUFFIX.sub('', name).strip()


def safe_link_name(name):
    """Create a portable URDF and STL name while preserving Unicode."""
    cleaned = UNSAFE_LINK_CHARACTERS.sub('_', logical_name(name))
    cleaned = cleaned.strip('._')
    return cleaned or 'link'


class Joint:
    def __init__(
        self,
        name,
        xyz,
        axis,
        parent,
        child,
        joint_type,
        upper_limit,
        lower_limit,
    ):
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.axis = axis
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

    def make_joint_xml(self):
        joint = Element('joint')
        joint.attrib = {'name': self.name, 'type': self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {
            'xyz': ' '.join(str(value) for value in self.xyz),
            'rpy': '0 0 0',
        }
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link': self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link': self.child}
        if self.type in ('revolute', 'continuous', 'prismatic'):
            axis = SubElement(joint, 'axis')
            axis.attrib = {
                'xyz': ' '.join(str(value) for value in self.axis)
            }
        if self.type in ('revolute', 'prismatic'):
            limit = SubElement(joint, 'limit')
            limit.attrib = {
                'upper': str(self.upper_limit),
                'lower': str(self.lower_limit),
                'effort': '100',
                'velocity': '100',
            }

        self.joint_xml = '\n'.join(
            utils.prettify(joint).split('\n')[1:]
        )


def _top_level_owner(occurrence, top_occurrences):
    """Return the top-level occurrence index containing ``occurrence``."""
    occurrence_path = occurrence.fullPathName
    matches = []
    for index, top_occurrence in enumerate(top_occurrences):
        top_path = top_occurrence.fullPathName
        if (
            occurrence_path == top_path
            or occurrence_path.startswith(top_path + '+')
        ):
            matches.append((len(top_path), index))
    if not matches:
        return None
    return max(matches)[1]


def _normalise(vector, joint_name):
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise ValueError(
            'Fusion joint "{}" has a zero-length motion axis.'.format(
                logical_name(joint_name)
            )
        )
    return [
        0.0 if abs(value / length) < 5e-7 else round(value / length, 6)
        for value in vector
    ]


def _transform_point(matrix, point):
    """Apply a Fusion Matrix3D to a point."""
    origin, x_axis, y_axis, z_axis = utils.matrix_parts(matrix)
    return [
        origin[index]
        + point[0] * x_axis[index]
        + point[1] * y_axis[index]
        + point[2] * z_axis[index]
        for index in range(3)
    ]


def _joint_geometry_frame(joint, is_as_built=False):
    """Return the physical joint frame in root-assembly coordinates.

    Joints collected from the root component expose their new geometry
    transforms in root-component coordinates.  A JointOrigin fallback is
    local to its occurrence and is explicitly promoted to root coordinates.
    """
    if is_as_built:
        transform = getattr(joint, 'transform', None)
        if transform is not None:
            return utils.matrix_parts(transform)

        geometry = getattr(joint, 'geometry', None)
        if geometry is not None:
            try:
                origin = geometry.origin.asArray()
                return origin, None, None, None
            except Exception:
                pass
    else:
        # Component 2 is the URDF parent, so prefer its geometry frame.  The
        # two frames describe the same physical point for a valid Fusion
        # joint; geometryOneTransform is a compatibility fallback.
        for attribute in ('geometryTwoTransform', 'geometryOneTransform'):
            transform = getattr(joint, attribute, None)
            if transform is not None:
                return utils.matrix_parts(transform)

        for suffix in ('Two', 'One'):
            geometry = getattr(
                joint, 'geometryOrOrigin' + suffix, None
            )
            occurrence = getattr(joint, 'occurrence' + suffix, None)
            if geometry is None:
                continue

            # JointOrigin.transform is the authoritative fallback added with
            # the geometry transform API. It is local to the occurrence.
            transform = getattr(geometry, 'transform', None)
            if transform is not None:
                origin, x_axis, y_axis, z_axis = utils.matrix_parts(
                    transform
                )
                if occurrence is not None:
                    world_origin = _transform_point(
                        occurrence.transform2, origin
                    )
                    _, occ_x, occ_y, occ_z = utils.matrix_parts(
                        occurrence.transform2
                    )

                    def rotate(axis):
                        return [
                            axis[0] * occ_x[index]
                            + axis[1] * occ_y[index]
                            + axis[2] * occ_z[index]
                            for index in range(3)
                        ]

                    return (
                        world_origin,
                        rotate(x_axis),
                        rotate(y_axis),
                        rotate(z_axis),
                    )
                return origin, x_axis, y_axis, z_axis

            # Legacy JointGeometry has an origin but no complete transform.
            # Joint objects owned by the root component return this point in
            # root-component coordinates.
            try:
                return geometry.origin.asArray(), None, None, None
            except Exception:
                continue

    raise ValueError(
        'Fusion joint "{}" has no usable joint geometry transform.'.format(
            logical_name(joint.name)
        )
    )


def _motion_axis(joint, motion, geometry_axes):
    """Return the motion axis in root-assembly coordinates."""
    joint_type = motion.jointType
    joint_types = adsk.fusion.JointTypes
    if joint_type == joint_types.RevoluteJointType:
        direction = getattr(motion, 'rotationAxis', None)
        vector = motion.rotationAxisVector.asArray()
    elif joint_type == joint_types.SliderJointType:
        direction = getattr(motion, 'slideDirection', None)
        vector = motion.slideDirectionVector.asArray()
    else:
        return [0.0, 0.0, 0.0]

    directions = getattr(adsk.fusion, 'JointDirections', None)
    if directions is not None and all(geometry_axes):
        direction_to_axis = {
            getattr(directions, 'XAxisJointDirection', object()):
                geometry_axes[0],
            getattr(directions, 'YAxisJointDirection', object()):
                geometry_axes[1],
            getattr(directions, 'ZAxisJointDirection', object()):
                geometry_axes[2],
        }
        if direction in direction_to_axis:
            vector = direction_to_axis[direction]

    return _normalise(vector, joint.name)


def _joint_motion(joint, geometry_axes=(None, None, None)):
    """Map Fusion's locale-independent joint enum to URDF data."""
    joint_type = joint.jointMotion.jointType
    joint_types = adsk.fusion.JointTypes
    axis = [0, 0, 0]
    upper_limit = 0.0
    lower_limit = 0.0

    if joint_type == joint_types.RigidJointType:
        urdf_type = 'fixed'
    elif joint_type == joint_types.RevoluteJointType:
        axis = _motion_axis(joint, joint.jointMotion, geometry_axes)
        limits = joint.jointMotion.rotationLimits
        maximum_enabled = limits.isMaximumValueEnabled
        minimum_enabled = limits.isMinimumValueEnabled
        if maximum_enabled != minimum_enabled:
            missing = 'lower' if maximum_enabled else 'upper'
            raise ValueError(
                'Fusion joint "{}" is missing its {} limit.'.format(
                    logical_name(joint.name), missing
                )
            )
        if maximum_enabled:
            urdf_type = 'revolute'
            upper_limit = round(limits.maximumValue, 6)
            lower_limit = round(limits.minimumValue, 6)
        else:
            urdf_type = 'continuous'
    elif joint_type == joint_types.SliderJointType:
        urdf_type = 'prismatic'
        axis = _motion_axis(joint, joint.jointMotion, geometry_axes)
        limits = joint.jointMotion.slideLimits
        maximum_enabled = limits.isMaximumValueEnabled
        minimum_enabled = limits.isMinimumValueEnabled
        if maximum_enabled != minimum_enabled:
            missing = 'lower' if maximum_enabled else 'upper'
            raise ValueError(
                'Fusion joint "{}" is missing its {} limit.'.format(
                    logical_name(joint.name), missing
                )
            )
        if maximum_enabled:
            upper_limit = round(limits.maximumValue / 100.0, 6)
            lower_limit = round(limits.minimumValue / 100.0, 6)
    else:
        raise ValueError(
            'Fusion joint "{}" has an unsupported joint type: {}.'.format(
                logical_name(joint.name), joint_type
            )
        )

    return {
        'type': urdf_type,
        'axis': axis,
        'upper_limit': upper_limit,
        'lower_limit': lower_limit,
    }


def _reachable_links(root_index, joints):
    """Return links reachable through directed parent-to-child joints."""
    reached = {root_index}
    changed = True
    while changed:
        changed = False
        for joint in joints:
            if (
                joint['parent_index'] in reached
                and joint['child_index'] not in reached
            ):
                reached.add(joint['child_index'])
                changed = True
    return reached


def _prefer_root_closing_joints(
    raw_joints, root_index, link_count
):
    """Choose removable joints touching the root as preferred loop edges.

    Candidates are considered from the end of Fusion's joint order. A joint is
    removed only when every link is still reachable from the root without it,
    so a bridge from base_link into the mechanism is never classified as a
    loop merely because it touches the root.
    """
    remaining = list(raw_joints)
    preferred_loops = []
    candidates = [
        joint
        for joint in reversed(raw_joints)
        if (
            joint['parent_index'] == root_index
            or joint['child_index'] == root_index
        )
    ]
    for candidate in candidates:
        trial = [joint for joint in remaining if joint is not candidate]
        if len(_reachable_links(root_index, trial)) == link_count:
            remaining = trial
            preferred_loops.append(candidate)

    preferred_loops.sort(key=lambda joint: joint['order'])
    return remaining, preferred_loops


def build_robot_model(root):
    """Collapse the design into a validated tree of top-level occurrences."""
    top_occurrences = list(root.occurrences)
    if not top_occurrences:
        raise ValueError('The Fusion design has no top-level components.')

    source_names = [
        logical_name(occurrence.name) for occurrence in top_occurrences
    ]
    raw_joints = []

    fusion_joints = [
        (joint, False) for joint in root.joints
    ]
    fusion_joints.extend(
        (joint, True)
        for joint in getattr(root, 'asBuiltJoints', ())
    )

    for order, (fusion_joint, is_as_built) in enumerate(fusion_joints):
        parent_index = _top_level_owner(
            fusion_joint.occurrenceTwo, top_occurrences
        )
        child_index = _top_level_owner(
            fusion_joint.occurrenceOne, top_occurrences
        )
        if parent_index is None or child_index is None:
            raise ValueError(
                'Fusion joint "{}" is not contained by a top-level '
                'component.'.format(logical_name(fusion_joint.name))
            )
        if parent_index == child_index:
            # The complete subassembly is one URDF link, so its internal
            # motion is intentionally folded away.
            continue

        (
            world_origin_cm,
            frame_x,
            frame_y,
            frame_z,
        ) = _joint_geometry_frame(fusion_joint, is_as_built)
        motion = _joint_motion(
            fusion_joint, (frame_x, frame_y, frame_z)
        )
        raw_joint = {
            'order': order,
            'source_name': logical_name(fusion_joint.name),
            'parent_index': parent_index,
            'child_index': child_index,
            'world_origin_cm': world_origin_cm,
            **motion,
        }
        raw_joint['world_axis'] = list(motion['axis'])
        raw_joints.append(raw_joint)

    named_roots = [
        index
        for index, name in enumerate(source_names)
        if name.casefold() == 'base_link'
    ]
    if len(named_roots) > 1:
        raise ValueError(
            'More than one top-level component resolves to "base_link": {}.'
            .format(', '.join(source_names[index] for index in named_roots))
        )

    if named_roots:
        root_index = named_roots[0]
    elif len(top_occurrences) == 1:
        root_index = 0
    else:
        child_indexes = {
            raw_joint['child_index'] for raw_joint in raw_joints
        }
        graph_roots = [
            index
            for index in range(len(top_occurrences))
            if index not in child_indexes
        ]
        if len(graph_roots) == 1:
            root_index = graph_roots[0]
        else:
            candidates = [
                source_names[index] for index in graph_roots
            ]
            raise ValueError(
                'The top-level components do not have one directed root. '
                'Name the intended root "base_link". Root candidates: {}.'
                .format(', '.join(candidates) or '(none)')
            )

    root_occurrence = top_occurrences[root_index]
    for raw_joint in raw_joints:
        raw_joint['xyz'] = [
            round(value / 100.0, 6)
            for value in utils.point_in_reference_frame(
                raw_joint['world_origin_cm'],
                root_occurrence.transform2,
            )
        ]
        if raw_joint['type'] != 'fixed':
            raw_joint['axis'] = _normalise(
                utils.vector_in_reference_frame(
                    raw_joint['axis'], root_occurrence.transform2
                ),
                raw_joint['source_name'],
            )

    tree_candidates, preferred_root_loops = (
        _prefer_root_closing_joints(
            raw_joints, root_index, len(top_occurrences)
        )
    )

    reached = {root_index}
    selected_joints = []
    omitted_joints = list(preferred_root_loops)
    pending_joints = list(tree_candidates)

    while pending_joints:
        next_pending = []
        added_link = False
        for raw_joint in pending_joints:
            parent_reached = raw_joint['parent_index'] in reached
            child_reached = raw_joint['child_index'] in reached

            if parent_reached and not child_reached:
                selected_joints.append(raw_joint)
                reached.add(raw_joint['child_index'])
                added_link = True
            elif parent_reached and child_reached:
                omitted_joints.append(raw_joint)
            else:
                next_pending.append(raw_joint)

        pending_joints = next_pending
        if not added_link:
            break

    if len(reached) != len(top_occurrences):
        unreachable = [
            source_names[index]
            for index in range(len(top_occurrences))
            if index not in reached
        ]
        blocked_joints = [
            raw_joint['source_name'] for raw_joint in pending_joints
        ]
        detail = ''
        if blocked_joints:
            detail = ' Blocked joints: {}.'.format(
                ', '.join(blocked_joints)
            )
        raise ValueError(
            'Disconnected or incorrectly directed top-level components: {}.{}'
            .format(', '.join(unreachable), detail)
        )

    # A final pass classifies closure edges that were deferred until their
    # parent became reachable in a later scan.
    for raw_joint in pending_joints:
        if (
            raw_joint['parent_index'] in reached
            and raw_joint['child_index'] in reached
        ):
            omitted_joints.append(raw_joint)
        else:
            raise ValueError(
                'Fusion joint "{}" cannot be directed from the selected root.'
                .format(raw_joint['source_name'])
            )

    selected_joints.sort(key=lambda joint: joint['order'])
    omitted_joints.sort(key=lambda joint: joint['order'])

    if len(selected_joints) != len(top_occurrences) - 1:
        candidates = [
            raw_joint['source_name'] for raw_joint in selected_joints
        ]
        raise ValueError(
            'Unable to build a URDF tree. Selected joints: {}.'.format(
                ', '.join(candidates) or '(none)'
            )
        )

    link_name_by_index = {}
    used_names = set()
    for index, source_name in enumerate(source_names):
        base_name = safe_link_name(source_name)
        link_name = base_name
        suffix = 2
        while link_name.casefold() in used_names:
            link_name = '{}_{}'.format(base_name, suffix)
            suffix += 1
        link_name_by_index[index] = link_name
        used_names.add(link_name.casefold())

    incoming_by_child = {
        raw_joint['child_index']: raw_joint
        for raw_joint in selected_joints
    }
    links = []
    for index, occurrence in enumerate(top_occurrences):
        incoming = incoming_by_child.get(index)
        links.append(
            {
                'name': link_name_by_index[index],
                'source_name': source_names[index],
                'occurrence': occurrence,
                'xyz': [0, 0, 0] if incoming is None else incoming['xyz'],
            }
        )

    joints = {}
    for number, raw_joint in enumerate(selected_joints, start=1):
        joint_name = 'joint_{}'.format(number)
        joints[joint_name] = {
            'source_name': raw_joint['source_name'],
            'type': raw_joint['type'],
            'axis': raw_joint['axis'],
            'upper_limit': raw_joint['upper_limit'],
            'lower_limit': raw_joint['lower_limit'],
            'parent': link_name_by_index[raw_joint['parent_index']],
            'child': link_name_by_index[raw_joint['child_index']],
            'xyz': raw_joint['xyz'],
        }

    loop_joints = []
    for number, raw_joint in enumerate(
        omitted_joints, start=len(selected_joints) + 1
    ):
        parent_occurrence = top_occurrences[raw_joint['parent_index']]
        child_occurrence = top_occurrences[raw_joint['child_index']]
        parent_origin = utils.point_in_reference_frame(
            raw_joint['world_origin_cm'],
            parent_occurrence.transform2,
        )
        child_origin = utils.point_in_reference_frame(
            raw_joint['world_origin_cm'],
            child_occurrence.transform2,
        )
        parent_axis = (
            [0.0, 0.0, 0.0]
            if raw_joint['type'] == 'fixed'
            else _normalise(
                utils.vector_in_reference_frame(
                    raw_joint['world_axis'],
                    parent_occurrence.transform2,
                ),
                raw_joint['source_name'],
            )
        )
        loop_joints.append(
            {
                'name': 'joint_{}'.format(number),
                'source_name': raw_joint['source_name'],
                'type': raw_joint['type'],
                'parent_source_name': source_names[
                    raw_joint['parent_index']
                ],
                'child_source_name': source_names[
                    raw_joint['child_index']
                ],
                'parent': link_name_by_index[raw_joint['parent_index']],
                'child': link_name_by_index[raw_joint['child_index']],
                'parent_origin': [
                    round(value / 100.0, 6)
                    for value in parent_origin
                ],
                'child_origin': [
                    round(value / 100.0, 6)
                    for value in child_origin
                ],
                'axis': parent_axis,
                'upper_limit': raw_joint['upper_limit'],
                'lower_limit': raw_joint['lower_limit'],
            }
        )

    return {
        'links': links,
        'joints': joints,
        'loop_joints': loop_joints,
        'root_occurrence': root_occurrence,
    }
