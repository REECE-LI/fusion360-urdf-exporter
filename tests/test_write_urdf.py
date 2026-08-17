import sys
import struct
import tempfile
import types
import unittest
from pathlib import Path
from xml.etree import ElementTree
from unittest import mock


# Autodesk modules only exist inside Fusion 360. These constants and simple
# classes cover the locale-independent API surface used by the exporter.
adsk = types.ModuleType('adsk')
adsk.core = types.ModuleType('adsk.core')
adsk.fusion = types.ModuleType('adsk.fusion')
adsk.fusion.JointTypes = types.SimpleNamespace(
    RigidJointType=0,
    RevoluteJointType=1,
    SliderJointType=2,
    CylindricalJointType=3,
)
adsk.fusion.CalculationAccuracy = types.SimpleNamespace(
    VeryHighCalculationAccuracy=99
)
adsk.fusion.MeshRefinementSettings = types.SimpleNamespace(
    MeshRefinementLow=1,
    MeshRefinementHigh=2,
)
adsk.fusion.DistanceUnits = types.SimpleNamespace(
    MillimeterDistanceUnits=0
)
adsk.fusion.JointOrigin = type('JointOrigin', (), {})
sys.modules.setdefault('adsk', adsk)
sys.modules.setdefault('adsk.core', adsk.core)
sys.modules.setdefault('adsk.fusion', adsk.fusion)

from Fusion_URDF_Exporter_ROS2.core import Joint, Link, Write
from Fusion_URDF_Exporter_ROS2 import Fusion_URDF_Exporter_ROS2 as Exporter
from Fusion_URDF_Exporter_ROS2.utils import utils


class Vector:
    def __init__(self, values):
        self.values = values

    def asArray(self):
        return list(self.values)


class Origin:
    def __init__(self, values):
        self.origin = Vector(values)


class Transform:
    def __init__(
        self,
        translation=(0, 0, 0),
        x_axis=(1, 0, 0),
        y_axis=(0, 1, 0),
        z_axis=(0, 0, 1),
    ):
        self.translation = Vector(translation)
        x, y, z = translation
        self.values = [
            x_axis[0], y_axis[0], z_axis[0], x,
            x_axis[1], y_axis[1], z_axis[1], y,
            x_axis[2], y_axis[2], z_axis[2], z,
            0, 0, 0, 1,
        ]

    def asArray(self):
        return list(self.values)


class PhysicalProperties:
    def __init__(self, mass=1.0):
        self.mass = mass
        self.centerOfMass = Vector([1.0, 2.0, 3.0])

    def getXYZMomentsOfInertia(self):
        return (True, 10000, 20000, 30000, 1000, 2000, 3000)


class Occurrence:
    def __init__(
        self,
        name,
        full_path=None,
        mass=1.0,
        transform=None,
        assembly_context=None,
    ):
        self.name = name
        self.fullPathName = full_path or name
        self.transform = transform or Transform()
        self.transform2 = self.transform
        self.assemblyContext = assembly_context
        self.component = types.SimpleNamespace(name=name)
        self.properties = PhysicalProperties(mass)

    def getPhysicalProperties(self, accuracy):
        self.requested_accuracy = accuracy
        return self.properties


class Limits:
    def __init__(self, minimum=None, maximum=None):
        self.isMinimumValueEnabled = minimum is not None
        self.isMaximumValueEnabled = maximum is not None
        self.minimumValue = minimum or 0.0
        self.maximumValue = maximum or 0.0


class Motion:
    def __init__(
        self,
        joint_type,
        axis=(0, 0, 1),
        minimum=None,
        maximum=None,
    ):
        self.jointType = joint_type
        self.rotationAxisVector = Vector(axis)
        self.slideDirectionVector = Vector(axis)
        self.rotationLimits = Limits(minimum, maximum)
        self.slideLimits = Limits(minimum, maximum)


class FusionJoint:
    def __init__(
        self,
        name,
        child,
        parent,
        joint_type=0,
        position=(0, 0, 0),
        axis=(0, 0, 1),
        minimum=None,
        maximum=None,
        geometry_transform=None,
    ):
        self.name = name
        self.occurrenceOne = child
        self.occurrenceTwo = parent
        self.jointMotion = Motion(
            joint_type, axis, minimum, maximum
        )
        self.geometryOrOriginOne = Origin(position)
        self.geometryOrOriginTwo = Origin(position)
        self.geometryOneTransform = geometry_transform
        self.geometryTwoTransform = geometry_transform


class Root:
    def __init__(self, occurrences, joints=(), as_built_joints=()):
        self.occurrences = occurrences
        self.joints = joints
        self.asBuiltJoints = as_built_joints


class RobotModelTest(unittest.TestCase):
    def test_nested_occurrence_transform_is_composed_to_root(self):
        assembly = Occurrence(
            'subassembly',
            transform=Transform(translation=(10, 0, 0)),
        )
        nested = Occurrence(
            'part',
            transform=Transform(translation=(0, 5, 0)),
            assembly_context=assembly,
        )

        frame = utils.frame_in_root_frame(
            (
                [1, 2, 3],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ),
            nested,
        )

        self.assertEqual(frame[0], [11, 7, 3])

    def test_custom_motion_axis_is_promoted_from_parent_occurrence(self):
        base = Occurrence('base_link')
        parent = Occurrence(
            'parent',
            transform=Transform(
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        child = Occurrence('child')
        model = Joint.build_robot_model(
            Root(
                [base, parent, child],
                [
                    FusionJoint('固定', parent, base),
                    FusionJoint(
                        '轴',
                        child,
                        parent,
                        joint_type=1,
                        axis=(1, 0, 0),
                    ),
                ],
            )
        )

        self.assertEqual(model['joints']['轴']['axis'], [0.0, 1.0, 0.0])

    def test_exact_base_link_is_preferred(self):
        base = Occurrence('base_link')
        child = Occurrence('child')
        model = Joint.build_robot_model(
            Root([base, child], [FusionJoint('fixed', child, base)])
        )

        self.assertEqual(model['links'][0]['name'], 'base_link')
        self.assertEqual(model['links'][1]['name'], 'child')

    def test_version_suffix_is_removed_and_chinese_link_names_are_kept(self):
        base = Occurrence('base_link v4:1', mass=5.0)
        rigid_link = Occurrence('左侧刚性件 v2:1')
        rotating_link = Occurrence('旋转臂 v3:1')
        root = Root(
            [base, rigid_link, rotating_link],
            [
                FusionJoint('刚性', rigid_link, base),
                FusionJoint(
                    '旋转',
                    rotating_link,
                    rigid_link,
                    joint_type=1,
                    axis=(0, 1, 0),
                ),
            ],
        )

        model = Joint.build_robot_model(root)

        self.assertEqual(
            [link['name'] for link in model['links']],
            ['base_link', '左侧刚性件', '旋转臂'],
        )
        self.assertEqual(
            [link['source_name'] for link in model['links']],
            ['base_link', '左侧刚性件', '旋转臂'],
        )
        self.assertEqual(list(model['joints']), ['刚性', '旋转'])
        self.assertEqual(model['joints']['刚性']['type'], 'fixed')
        self.assertEqual(model['joints']['刚性']['source_name'], '刚性')
        self.assertEqual(model['joints']['旋转']['type'], 'continuous')
        self.assertEqual(model['joints']['旋转']['source_name'], '旋转')

    def test_unique_graph_root_is_used_when_base_name_is_absent(self):
        root_link = Occurrence('中文机架 v1:1')
        child = Occurrence('子零件 v1:1')
        model = Joint.build_robot_model(
            Root(
                [child, root_link],
                [FusionJoint('固定', child, root_link)],
            )
        )

        self.assertEqual(model['links'][0]['name'], '子零件')
        self.assertEqual(model['links'][1]['name'], '中文机架')
        self.assertEqual(
            model['joints']['固定']['parent'], '中文机架'
        )
        self.assertEqual(model['joints']['固定']['child'], '子零件')

    def test_single_top_level_subassembly_needs_no_joint(self):
        only = Occurrence('完整机器人 v7:1')
        model = Joint.build_robot_model(Root([only]))

        self.assertEqual(model['links'][0]['name'], '完整机器人')
        self.assertEqual(model['joints'], {})

    def test_unsafe_characters_and_duplicate_names_are_disambiguated(self):
        base = Occurrence('base_link')
        first = Occurrence('左 臂 v1:1')
        second = Occurrence('左 臂 v2:1')
        model = Joint.build_robot_model(
            Root(
                [base, first, second],
                [
                    FusionJoint('A', first, base),
                    FusionJoint('B', second, first),
                ],
            )
        )

        self.assertEqual(
            [link['name'] for link in model['links']],
            ['base_link', '左_臂', '左_臂_2'],
        )

    def test_joint_names_follow_fusion_names_and_are_disambiguated(self):
        base = Occurrence('base_link')
        first = Occurrence('first')
        second = Occurrence('second')
        third = Occurrence('third')
        model = Joint.build_robot_model(
            Root(
                [base, first, second, third],
                [
                    FusionJoint('左/关节 v4:1', first, base),
                    FusionJoint('左 关节', second, first),
                    FusionJoint('左 关节', third, second),
                ],
            )
        )

        self.assertEqual(
            list(model['joints']),
            ['左_关节', '左_关节_2', '左_关节_3'],
        )
        self.assertEqual(
            [joint['source_name'] for joint in model['joints'].values()],
            ['左/关节', '左 关节', '左 关节'],
        )

    def test_internal_subassembly_joint_is_folded_away(self):
        assembly = Occurrence('总成 v1:1')
        nested_a = Occurrence(
            '子件A:1', '总成 v1:1+子件A:1'
        )
        nested_b = Occurrence(
            '子件B:1', '总成 v1:1+子件B:1'
        )
        model = Joint.build_robot_model(
            Root(
                [assembly],
                [FusionJoint('内部旋转', nested_a, nested_b, 1)],
            )
        )

        self.assertEqual(len(model['links']), 1)
        self.assertEqual(model['joints'], {})

    def test_slider_limits_convert_from_centimetres(self):
        base = Occurrence('base')
        child = Occurrence('slider')
        model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint(
                        '滑动',
                        child,
                        base,
                        joint_type=2,
                        axis=(1, 0, 0),
                        minimum=-10,
                        maximum=25,
                    )
                ],
            )
        )

        joint = model['joints']['滑动']
        self.assertEqual(joint['type'], 'prismatic')
        self.assertEqual(joint['lower_limit'], -0.1)
        self.assertEqual(joint['upper_limit'], 0.25)

    def test_joint_positions_are_expressed_in_base_component_frame(self):
        base = Occurrence(
            'base_link',
            transform=Transform(
                translation=(10, 20, 0),
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        child = Occurrence('child')
        model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint(
                        '连接',
                        child,
                        base,
                        # JointOrigin coordinates are local to Component 2.
                        position=(10, 0, 0),
                    )
                ],
            )
        )

        self.assertEqual(model['links'][0]['xyz'], [0, 0, 0])
        self.assertEqual(model['links'][1]['xyz'], [0.1, 0.0, 0.0])
        self.assertIs(model['root_occurrence'], base)

    def test_disconnected_tree_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'directed root'):
            Joint.build_robot_model(
                Root([Occurrence('A'), Occurrence('B')])
            )

    def test_named_base_with_incoming_joint_is_rejected(self):
        base = Occurrence('base_link v2:1')
        other = Occurrence('other')
        with self.assertRaisesRegex(
            ValueError, 'Disconnected or incorrectly directed'
        ):
            Joint.build_robot_model(
                Root(
                    [base, other],
                    [FusionJoint('方向错误', base, other)],
                )
            )

    def test_later_duplicate_parent_joint_becomes_loop_constraint(self):
        base = Occurrence('base_link')
        second_parent = Occurrence('second_parent')
        child = Occurrence('child')
        model = Joint.build_robot_model(
            Root(
                [base, second_parent, child],
                [
                    FusionJoint('支路', second_parent, base),
                    FusionJoint('旋转 4', child, base, 1),
                    FusionJoint('旋转 5', child, second_parent, 1),
                ],
            )
        )

        self.assertEqual(len(model['joints']), 2)
        self.assertEqual(
            [joint['source_name'] for joint in model['joints'].values()],
            ['支路', '旋转 5'],
        )
        self.assertEqual(
            [
                joint['source_name']
                for joint in model['loop_joints']
            ],
            ['旋转 4'],
        )
        self.assertEqual(
            model['loop_joints'][0]['child_source_name'],
            'child',
        )

    def test_joint_pointing_back_to_named_base_becomes_loop_constraint(self):
        base = Occurrence('base_link')
        first = Occurrence('first')
        second = Occurrence('second')
        model = Joint.build_robot_model(
            Root(
                [base, first, second],
                [
                    FusionJoint('A', first, base),
                    FusionJoint('B', second, first),
                    FusionJoint('闭环', base, second),
                ],
            )
        )

        self.assertEqual(len(model['joints']), 2)
        self.assertEqual(
            [
                joint['source_name']
                for joint in model['loop_joints']
            ],
            ['闭环'],
        )

    def test_fusion_order_is_preserved_for_parallel_joints(self):
        base = Occurrence('base_link')
        child = Occurrence('child')

        first_model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint('旋转 4', child, base, 1),
                    FusionJoint('旋转 5', child, base, 1),
                ],
            )
        )
        second_model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint('旋转 5', child, base, 1),
                    FusionJoint('旋转 4', child, base, 1),
                ],
            )
        )

        self.assertEqual(
            first_model['joints']['旋转_4']['source_name'], '旋转 4'
        )
        self.assertEqual(
            first_model['loop_joints'][0]['source_name'], '旋转 5'
        )
        self.assertEqual(
            second_model['joints']['旋转_5']['source_name'], '旋转 5'
        )
        self.assertEqual(
            second_model['loop_joints'][0]['source_name'], '旋转 4'
        )

    def test_multiple_closed_chain_edges_are_exported_separately(self):
        base = Occurrence('base_link')
        first = Occurrence('first')
        second = Occurrence('second')
        model = Joint.build_robot_model(
            Root(
                [base, first, second],
                [
                    FusionJoint('A', first, base),
                    FusionJoint('B', second, base),
                    FusionJoint('C', second, first),
                    FusionJoint('D', first, second),
                ],
            )
        )

        self.assertEqual(len(model['joints']), 2)
        self.assertEqual(
            [
                joint['source_name']
                for joint in model['loop_joints']
            ],
            ['B', 'D'],
        )

    def test_redundant_joint_touching_base_is_preferred_as_loop(self):
        base = Occurrence('base_link')
        first = Occurrence('first')
        second = Occurrence('second')
        model = Joint.build_robot_model(
            Root(
                [base, first, second],
                [
                    FusionJoint('较早的闭环', second, first),
                    FusionJoint('树关节 A', first, base),
                    FusionJoint('树关节 B', second, base),
                    FusionJoint('较晚的闭环', first, second),
                ],
            )
        )

        self.assertEqual(
            [joint['source_name'] for joint in model['joints'].values()],
            ['较早的闭环', '树关节 A'],
        )
        self.assertEqual(
            [joint['name'] for joint in model['loop_joints']],
            ['树关节_B', '较晚的闭环'],
        )
        self.assertEqual(
            [joint['source_name'] for joint in model['loop_joints']],
            ['树关节 B', '较晚的闭环'],
        )

    def test_loop_origin_uses_joint_geometry_not_occurrence_relative_pose(self):
        base = Occurrence('base_link')
        branch = Occurrence('branch')
        child = Occurrence(
            'left_link_3',
            transform=Transform(translation=(-5, 0, 2)),
        )
        model = Joint.build_robot_model(
            Root(
                [base, branch, child],
                [
                    FusionJoint('A', branch, base),
                    FusionJoint('B', child, branch),
                    FusionJoint(
                        '闭环轴孔',
                        child,
                        base,
                        joint_type=1,
                        # Deliberately wrong legacy point: the new geometry
                        # transform must take precedence.
                        position=(-0.9631, 0, 0),
                        geometry_transform=Transform(
                            # Component 2 is the base occurrence, so this
                            # local frame is already the robot root frame.
                            translation=(-7.2803, 0, 2.8171)
                        ),
                    ),
                ],
            )
        )

        loop = model['loop_joints'][0]
        self.assertEqual(
            loop['parent_origin'], [-0.072803, 0.0, 0.028171]
        )
        self.assertEqual(
            loop['child_origin'], [-0.072803, 0.0, 0.028171]
        )

    def test_loop_origins_use_exported_link_frames_not_occurrence_frames(self):
        base = Occurrence('base_link')
        middle = Occurrence('middle')
        last = Occurrence(
            'last',
            transform=Transform(
                translation=(10, 20, 0),
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        model = Joint.build_robot_model(
            Root(
                [base, middle, last],
                [
                    FusionJoint(
                        'A', middle, base, position=(10, 0, 0)
                    ),
                    FusionJoint('B', last, middle),
                    FusionJoint(
                        '闭环',
                        base,
                        last,
                        joint_type=1,
                        geometry_transform=Transform(
                            # geometryTwoTransform is already in the root
                            # assembly frame; applying last.transform2 again
                            # would rotate this physical point incorrectly.
                            translation=(10, 30, 0)
                        ),
                    ),
                ],
            )
        )

        loop = model['loop_joints'][0]
        self.assertEqual(loop['parent_origin'], [0.1, 0.3, 0.0])
        self.assertEqual(loop['child_origin'], [0.1, 0.3, 0.0])

    def test_geometry_transform_is_not_promoted_twice(self):
        base = Occurrence('base_link')
        child = Occurrence(
            'child',
            transform=Transform(
                translation=(10, 20, 0),
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint(
                        '连接',
                        child,
                        base,
                        geometry_transform=Transform(
                            translation=(10, 30, 0)
                        ),
                    )
                ],
            )
        )

        self.assertEqual(model['links'][1]['xyz'], [0.1, 0.3, 0.0])

    def test_loop_metadata_separates_root_and_base_link_frames(self):
        base = Occurrence(
            'base_link',
            transform=Transform(
                translation=(10, 20, 0),
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        child = Occurrence('child')
        model = Joint.build_robot_model(
            Root(
                [base, child],
                [
                    FusionJoint('树关节', child, base),
                    FusionJoint(
                        '闭环',
                        child,
                        base,
                        joint_type=1,
                        geometry_transform=Transform(
                            translation=(10, 20, 0)
                        ),
                    ),
                ],
            )
        )

        loop = model['loop_joints'][0]
        self.assertEqual(loop['parent_origin'], [0.0, 0.0, 0.0])
        self.assertEqual(loop['child_origin'], [0.0, 0.0, 0.0])
        self.assertEqual(loop['world_origin'], [0.1, 0.2, 0.0])

    def test_unsupported_joint_type_is_rejected(self):
        first = Occurrence('first')
        second = Occurrence('second')
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            Joint.build_robot_model(
                Root(
                    [first, second],
                    [FusionJoint('圆柱', second, first, 3)],
                )
            )


class WriteUrdfTest(unittest.TestCase):
    def test_all_urdf_writes_explicitly_use_utf8(self):
        occurrence = Occurrence('中文总成')
        model = Joint.build_robot_model(Root([occurrence]))
        inertial = Link.make_inertial_dict(model['links'])
        real_open = open
        encodings = []

        def checked_open(*args, **kwargs):
            mode = kwargs.get('mode', 'r')
            if 'w' in mode or 'a' in mode:
                encodings.append(kwargs.get('encoding'))
            return real_open(*args, **kwargs)

        with tempfile.TemporaryDirectory() as temp_dir:
            with mock.patch('builtins.open', side_effect=checked_open):
                Write.write_urdf(
                    model['links'],
                    model['joints'],
                    model['loop_joints'],
                    inertial,
                    'robot',
                    temp_dir,
                )

        self.assertTrue(encodings)
        self.assertTrue(
            all(encoding == 'utf-8' for encoding in encodings)
        )

    def test_writes_source_link_names_and_physical_values(self):
        base_occurrence = Occurrence('底座 v4:1', mass=2.0)
        child_occurrence = Occurrence('旋转臂 v2:1')
        model = Joint.build_robot_model(
            Root(
                [base_occurrence, child_occurrence],
                [
                    FusionJoint(
                        '旋转',
                        child_occurrence,
                        base_occurrence,
                        joint_type=1,
                        minimum=-1.5,
                        maximum=1.5,
                    )
                ],
            )
        )
        inertial = Link.make_inertial_dict(model['links'])

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Write.write_urdf(
                model['links'],
                model['joints'],
                model['loop_joints'],
                inertial,
                'robot',
                temp_dir,
            )
            text = Path(output_path).read_text(encoding='utf-8')
            root = ElementTree.fromstring(text)

        self.assertEqual(root.attrib['name'], 'robot')
        self.assertIn('<!-- Fusion component: 底座 -->', text)
        self.assertIn('<!-- Fusion joint: 旋转 -->', text)
        self.assertEqual(
            {link.attrib['name'] for link in root.findall('link')},
            {'底座', '旋转臂'},
        )
        self.assertEqual(
            root.find("./joint[@name='旋转']").attrib['type'],
            'revolute',
        )
        self.assertEqual(
            root.find("./link[@name='底座']/inertial/mass").attrib[
                'value'
            ],
            '2.0',
        )
        mesh_names = [
            mesh.attrib['filename'] for mesh in root.findall('.//mesh')
        ]
        self.assertEqual(
            set(mesh_names),
            {'meshes/底座.stl', 'meshes/旋转臂.stl'},
        )
        self.assertNotIn(' v4:1', text)

    def test_writes_closed_chain_origins_in_both_local_frames(self):
        base = Occurrence('base_link')
        branch = Occurrence(
            '左支路', transform=Transform(translation=(5, 0, 0))
        )
        child = Occurrence(
            'left_link_3', transform=Transform(translation=(0, 5, 0))
        )
        model = Joint.build_robot_model(
            Root(
                [base, branch, child],
                [
                    FusionJoint(
                        '旋转 3', branch, base, 1,
                        position=(10, 0, 0),
                    ),
                    FusionJoint(
                        '旋转 4', child, branch, 1,
                        position=(10, 10, 0),
                    ),
                    FusionJoint(
                        '旋转 5', base, child, 1,
                        position=(10, 10, 0),
                    ),
                ],
            )
        )
        inertial = Link.make_inertial_dict(model['links'])

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Write.write_urdf(
                model['links'],
                model['joints'],
                model['loop_joints'],
                inertial,
                'robot',
                temp_dir,
            )
            text = Path(output_path).read_text(encoding='utf-8')
            ElementTree.fromstring(text)

        root = ElementTree.fromstring(text)
        self.assertEqual(len(root.findall('joint')), 2)
        self.assertIn('<!-- Fusion loop joint: 旋转 5 -->', text)
        self.assertIn(
            '<!-- Non-standard URDF metadata: recreate this closure',
            text,
        )
        loop = root.find("./loop_joint[@name='旋转_5']")
        self.assertEqual(
            loop.find('parent_origin').attrib['xyz'],
            '-0.05 0.05 0.0',
        )
        self.assertEqual(
            loop.find('child_origin').attrib['xyz'],
            '0.1 0.15 0.0',
        )
        self.assertEqual(
            loop.find('world_origin').attrib['xyz'],
            '0.1 0.15 0.0',
        )
        self.assertEqual(loop.find('world_origin').attrib['frame'], 'root')
        self.assertEqual(loop.find('axis').attrib['frame'], 'parent')
        self.assertEqual(loop.find('world_axis').attrib['frame'], 'root')


class ExportStlTest(unittest.TestCase):
    @staticmethod
    def binary_triangle():
        return (
            bytes(80)
            + struct.pack('<I', 1)
            + struct.pack(
                '<12fH',
                0, 0, 1,
                0, 0, 0,
                1, 0, 0,
                0, 1, 0,
                0,
            )
        )

    def test_exports_one_file_per_top_level_occurrence_and_clears_stale_mesh(self):
        class Options:
            def __init__(self, occurrence, filename):
                self.geometry = occurrence
                self.filename = filename

        class ExportManager:
            def __init__(self):
                self.options = []

            def createSTLExportOptions(self, occurrence, filename):
                option = Options(occurrence, filename)
                self.options.append(option)
                return option

            def execute(self, option):
                Path(option.filename).write_bytes(
                    ExportStlTest.binary_triangle()
                )
                return True

        manager = ExportManager()
        design = types.SimpleNamespace(exportManager=manager)
        links = [
            {
                'name': 'base_link',
                'source_name': '总成',
                'occurrence': Occurrence('总成'),
            },
            {
                'name': 'link_1',
                'source_name': '车轮',
                'occurrence': Occurrence('车轮'),
            },
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            mesh_dir = Path(temp_dir) / 'meshes'
            mesh_dir.mkdir()
            (mesh_dir / 'old_child.stl').write_bytes(b'old')

            utils.export_stl(
                design, temp_dir, links, links[0]['occurrence']
            )

            self.assertEqual(
                sorted(path.name for path in mesh_dir.iterdir()),
                ['base_link.stl', 'link_1.stl'],
            )

        self.assertEqual(len(manager.options), 2)
        self.assertIs(
            manager.options[0].geometry,
            links[0]['occurrence'].component,
        )
        self.assertIs(
            manager.options[1].geometry,
            links[1]['occurrence'].component,
        )
        self.assertTrue(
            all(option.isOneFilePerBody is False
                for option in manager.options)
        )
        self.assertTrue(
            all(option.unitType == 0 for option in manager.options)
        )
        self.assertTrue(
            all(option.meshRefinement == 2 for option in manager.options)
        )

    def test_component_meshes_are_baked_into_base_component_frame(self):
        class Options:
            def __init__(self, geometry, filename):
                self.geometry = geometry
                self.filename = filename

        class ExportManager:
            def createSTLExportOptions(self, geometry, filename):
                return Options(geometry, filename)

            def execute(self, option):
                Path(option.filename).write_bytes(
                    ExportStlTest.binary_triangle()
                )
                return True

        base = Occurrence(
            'base_link',
            transform=Transform(
                translation=(10, 20, 0),
                x_axis=(0, 1, 0),
                y_axis=(-1, 0, 0),
                z_axis=(0, 0, 1),
            ),
        )
        child = Occurrence(
            'child',
            transform=Transform(translation=(20, 20, 0)),
        )
        links = [
            {
                'name': 'base_link',
                'source_name': 'base_link',
                'occurrence': base,
            },
            {
                'name': 'child',
                'source_name': 'child',
                'occurrence': child,
            },
        ]
        design = types.SimpleNamespace(
            exportManager=ExportManager()
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            utils.export_stl(design, temp_dir, links, base)
            base_data = (
                Path(temp_dir) / 'meshes' / 'base_link.stl'
            ).read_bytes()
            child_data = (
                Path(temp_dir) / 'meshes' / 'child.stl'
            ).read_bytes()

        base_values = struct.unpack_from('<12f', base_data, 84)
        child_values = struct.unpack_from('<12f', child_data, 84)
        self.assertEqual(
            base_values[3:12],
            (
                0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
            ),
        )
        self.assertEqual(
            child_values[3:12],
            (
                0.0, -100.0, 0.0,
                0.0, -101.0, 0.0,
                1.0, -100.0, 0.0,
            ),
        )


class SuccessMessageTest(unittest.TestCase):
    def test_lists_embedded_closed_chain_constraints(self):
        message = Exporter.build_success_message(
            '/tmp/robot',
            'robot',
            [
                {'source_name': '旋转 5'},
                {'source_name': '刚性闭环'},
            ],
        )

        self.assertIn('/tmp/robot/robot.urdf', message)
        self.assertNotIn('robot.loop_joints.xml', message)
        self.assertIn(
            'Closed-chain metadata (for MuJoCo equality/connect conversion)',
            message,
        )
        self.assertIn('旋转 5', message)
        self.assertIn('刚性闭环', message)

    def test_omits_closed_chain_section_for_normal_tree(self):
        message = Exporter.build_success_message(
            '/tmp/robot', 'robot', []
        )

        self.assertNotIn(
            'Closed-chain metadata (for MuJoCo equality/connect conversion)',
            message,
        )


if __name__ == '__main__':
    unittest.main()
