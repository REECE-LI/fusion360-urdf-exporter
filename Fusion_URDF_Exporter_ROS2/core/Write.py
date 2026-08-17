# -*- coding: utf-8 -*-
"""Write a portable URDF file from data extracted from Fusion 360."""

import os
from xml.etree.ElementTree import Element, SubElement

from . import Joint, Link
from ..utils import utils


def _xml_comment(label, source_name):
    """Return a safe single-line XML comment."""
    safe_name = (
        str(source_name)
        .replace('--', '—')
        .replace('\n', ' ')
        .replace('\r', ' ')
    )
    if safe_name.endswith('-'):
        safe_name += ' '
    return '<!-- {}: {} -->\n'.format(label, safe_name)


def write_link_urdf(links, links_xyz_dict, file_name, inertial_dict):
    """Append all URDF links and populate their assembly positions."""
    with open(file_name, mode='a', encoding='utf-8') as urdf_file:
        for link_data in links:
            name = link_data['name']
            center_of_mass = [
                center - origin
                for center, origin in zip(
                    inertial_dict[name]['center_of_mass'], link_data['xyz']
                )
            ]
            link = Link.Link(
                name=name,
                xyz=link_data['xyz'],
                center_of_mass=center_of_mass,
                mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'],
            )
            links_xyz_dict[link.name] = link_data['xyz']
            link.make_link_xml()
            urdf_file.write(
                _xml_comment('Fusion component', link_data['source_name'])
            )
            urdf_file.write(link.link_xml)
            urdf_file.write('\n')


def write_joint_urdf(joints_dict, links_xyz_dict, file_name):
    """Append all URDF joints."""
    with open(file_name, mode='a', encoding='utf-8') as urdf_file:
        for name, joint_data in joints_dict.items():
            parent = joint_data['parent']
            child = joint_data['child']
            try:
                xyz = [
                    round(joint_xyz - reference_xyz, 6)
                    for joint_xyz, reference_xyz in zip(
                        joint_data['xyz'],
                        links_xyz_dict[parent],
                    )
                ]
            except KeyError as error:
                raise ValueError(
                    'Joint "{}" references a missing link "{}". '
                    'Fusion Component 2 must be the parent and Component 1 '
                    'must be the child.'.format(name, error.args[0])
                ) from error

            joint = Joint.Joint(
                name=name,
                joint_type=joint_data['type'],
                xyz=xyz,
                axis=joint_data['axis'],
                parent=parent,
                child=child,
                upper_limit=joint_data['upper_limit'],
                lower_limit=joint_data['lower_limit'],
            )
            joint.make_joint_xml()
            urdf_file.write(
                _xml_comment('Fusion joint', joint_data['source_name'])
            )
            urdf_file.write(joint.joint_xml)
            urdf_file.write('\n')


def _format_vector(values):
    return ' '.join(str(round(value, 6)) for value in values)


def write_loop_joints(loop_joints, file_name):
    """Append non-standard closed-chain metadata to the robot XML."""
    if not loop_joints:
        return

    with open(file_name, mode='a', encoding='utf-8') as output:
        for joint_data in loop_joints:
            output.write(
                _xml_comment(
                    'Fusion loop joint', joint_data['source_name']
                )
            )
            output.write(
                '<!-- Non-standard URDF metadata: recreate this closure '
                'as a simulator constraint; it is not a body joint. -->\n'
            )
            joint = Element(
                'loop_joint',
                {
                    'name': joint_data['name'],
                    'type': joint_data['type'],
                },
            )
            SubElement(joint, 'parent', {'link': joint_data['parent']})
            SubElement(joint, 'child', {'link': joint_data['child']})
            SubElement(
                joint,
                'parent_origin',
                {'xyz': _format_vector(joint_data['parent_origin'])},
            )
            SubElement(
                joint,
                'child_origin',
                {'xyz': _format_vector(joint_data['child_origin'])},
            )
            if 'world_origin' in joint_data:
                SubElement(
                    joint,
                    'world_origin',
                    {
                        'xyz': _format_vector(joint_data['world_origin']),
                        'frame': 'root',
                    },
                )
            if joint_data['type'] in (
                'revolute', 'continuous', 'prismatic'
            ):
                SubElement(
                    joint,
                    'axis',
                    {
                        'xyz': _format_vector(joint_data['axis']),
                        'frame': 'parent',
                    },
                )
                if 'world_axis' in joint_data:
                    SubElement(
                        joint,
                        'world_axis',
                        {
                            'xyz': _format_vector(
                                joint_data['world_axis']
                            ),
                            'frame': 'root',
                        },
                    )
            if joint_data['type'] in ('revolute', 'prismatic'):
                SubElement(
                    joint,
                    'limit',
                    {
                        'upper': str(joint_data['upper_limit']),
                        'lower': str(joint_data['lower_limit']),
                    },
                )

            xml = '\n'.join(
                utils.prettify(joint).split('\n')[1:]
            )
            output.write(xml)
            output.write('\n')


def write_urdf(
    links,
    joints_dict,
    loop_joints,
    inertial_dict,
    robot_name,
    save_dir,
):
    """Write ``<save_dir>/<robot_name>.urdf`` and return its path."""
    os.makedirs(save_dir, exist_ok=True)
    file_name = os.path.join(save_dir, robot_name + '.urdf')
    legacy_loop_file = os.path.join(
        save_dir, robot_name + '.loop_joints.xml'
    )
    if os.path.exists(legacy_loop_file):
        os.remove(legacy_loop_file)

    with open(file_name, mode='w', encoding='utf-8') as urdf_file:
        urdf_file.write('<?xml version="1.0" ?>\n')
        urdf_file.write('<robot name="{}">\n'.format(robot_name))
        urdf_file.write(
            '  <material name="silver">\n'
            '    <color rgba="0.700 0.700 0.700 1.000"/>\n'
            '  </material>\n'
        )

    links_xyz_dict = {}
    write_link_urdf(links, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, links_xyz_dict, file_name)
    write_loop_joints(loop_joints, file_name)

    with open(file_name, mode='a', encoding='utf-8') as urdf_file:
        urdf_file.write('</robot>\n')

    return file_name
