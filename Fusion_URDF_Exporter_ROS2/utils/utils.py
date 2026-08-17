# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import math
import os.path
import shutil
import struct
from xml.etree import ElementTree
from xml.dom import minidom


def matrix_parts(matrix):
    """Return origin and basis vectors from a Fusion Matrix3D."""
    try:
        origin, x_axis, y_axis, z_axis = (
            matrix.getAsCoordinateSystem()
        )
        return (
            origin.asArray(),
            x_axis.asArray(),
            y_axis.asArray(),
            z_axis.asArray(),
        )
    except (AttributeError, TypeError):
        values = matrix.asArray()
        return (
            [values[3], values[7], values[11]],
            [values[0], values[4], values[8]],
            [values[1], values[5], values[9]],
            [values[2], values[6], values[10]],
        )


def _rotate_by_frame(vector, frame_axes):
    """Rotate a vector from a local frame into its parent frame."""
    x_axis, y_axis, z_axis = frame_axes
    return [
        vector[0] * x_axis[index]
        + vector[1] * y_axis[index]
        + vector[2] * z_axis[index]
        for index in range(3)
    ]


def _compose_frames(parent_frame, local_frame):
    """Compose a local Fusion frame with its parent frame."""
    parent_origin, parent_x, parent_y, parent_z = parent_frame
    local_origin, local_x, local_y, local_z = local_frame
    parent_axes = (parent_x, parent_y, parent_z)
    origin = [
        parent_origin[index]
        + _rotate_by_frame(local_origin, parent_axes)[index]
        for index in range(3)
    ]
    return (
        origin,
        _rotate_by_frame(local_x, parent_axes),
        _rotate_by_frame(local_y, parent_axes),
        _rotate_by_frame(local_z, parent_axes),
    )


def occurrence_root_frame(occurrence):
    """Return an occurrence's local-to-root coordinate frame.

    ``Occurrence.transform2`` is expressed in the occurrence's assembly
    context.  For a nested occurrence, every assembly-context transform must
    be composed before a point or axis can be compared with root-level URDF
    link data.  The root component itself is represented by the identity
    frame, so a top-level occurrence needs only its own ``transform2``.
    """
    identity = (
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    )
    if occurrence is None:
        return identity

    chain = []
    current = occurrence
    seen = set()
    while current is not None:
        marker = id(current)
        if marker in seen:
            raise ValueError('Fusion occurrence assembly context contains a cycle.')
        seen.add(marker)
        chain.append(current)
        current = getattr(current, 'assemblyContext', None)

    frame = identity
    for item in reversed(chain):
        # ``transform`` is retired by Fusion and can return incorrect
        # results.  Do not silently fall back to it: a missing transform2
        # means the occurrence cannot be placed in the exporter frame.
        transform = getattr(item, 'transform2', None)
        if transform is None:
            raise ValueError(
                'Fusion occurrence has no transform2 in its assembly context.'
            )
        frame = _compose_frames(frame, matrix_parts(transform))
    return frame


def frame_in_root_frame(frame, occurrence):
    """Promote a local joint frame into the root component frame."""
    return _compose_frames(occurrence_root_frame(occurrence), frame)


def vector_in_root_frame(vector, occurrence):
    """Promote a local direction vector into the root component frame."""
    _, x_axis, y_axis, z_axis = occurrence_root_frame(occurrence)
    return _rotate_by_frame(vector, (x_axis, y_axis, z_axis))


def point_in_reference_frame(point, reference_matrix, scale=1.0):
    """Express an assembly-space point in a reference occurrence frame."""
    origin, x_axis, y_axis, z_axis = matrix_parts(reference_matrix)
    delta = [
        point[index] - origin[index] * scale
        for index in range(3)
    ]
    return [
        sum(delta[index] * axis[index] for index in range(3))
        for axis in (x_axis, y_axis, z_axis)
    ]


def vector_in_reference_frame(vector, reference_matrix):
    """Express an assembly-space direction in a reference occurrence frame."""
    _, x_axis, y_axis, z_axis = matrix_parts(reference_matrix)
    return [
        sum(vector[index] * axis[index] for index in range(3))
        for axis in (x_axis, y_axis, z_axis)
    ]


def relative_occurrence_transform(reference, occurrence):
    """Return occurrence pose expressed in the reference local frame."""
    reference_origin, ref_x, ref_y, ref_z = matrix_parts(
        reference.transform2
    )
    origin, x_axis, y_axis, z_axis = matrix_parts(
        occurrence.transform2
    )
    relative_origin = point_in_reference_frame(
        origin, reference.transform2
    )
    relative_axes = []
    for axis in (x_axis, y_axis, z_axis):
        relative_axes.append(
            [
                sum(axis[index] * ref_axis[index]
                    for index in range(3))
                for ref_axis in (ref_x, ref_y, ref_z)
            ]
        )
    return relative_origin, relative_axes


def _normalise(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        return [0.0, 0.0, 0.0]
    return [value / length for value in vector]


def transform_binary_stl(file_name, translation_cm, axes):
    """Apply a rigid transform to a binary millimetre STL."""
    with open(file_name, mode='rb') as stl_file:
        data = stl_file.read()

    if len(data) < 84:
        raise ValueError('STL file is not a valid binary STL.')
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) < expected_size:
        raise ValueError('STL file has an invalid binary triangle table.')

    translation_mm = [value * 10.0 for value in translation_cm]
    x_axis, y_axis, z_axis = axes

    def rotate(vector):
        return [
            vector[0] * x_axis[index]
            + vector[1] * y_axis[index]
            + vector[2] * z_axis[index]
            for index in range(3)
        ]

    output_data = bytearray(data[:84])
    for triangle_index in range(triangle_count):
        offset = 84 + triangle_index * 50
        values = struct.unpack_from('<12f', data, offset)
        transformed = _normalise(rotate(values[0:3]))
        for vertex_offset in (3, 6, 9):
            vertex = rotate(
                values[vertex_offset:vertex_offset + 3]
            )
            transformed.extend(
                vertex[index] + translation_mm[index]
                for index in range(3)
            )
        output_data.extend(struct.pack('<12f', *transformed))
        output_data.extend(data[offset + 48:offset + 50])

    output_data.extend(data[expected_size:])
    with open(file_name, mode='wb') as stl_file:
        stl_file.write(output_data)


def export_stl(design, save_dir, links, root_occurrence):
    """Export each top-level component in its local frame as one STL."""
    mesh_dir = os.path.join(save_dir, 'meshes')
    if os.path.isdir(mesh_dir):
        shutil.rmtree(mesh_dir)
    os.makedirs(mesh_dir)

    export_manager = design.exportManager
    failures = []
    for link in links:
        file_name = os.path.join(mesh_dir, link['name'] + '.stl')
        try:
            # Export the component definition in its own coordinate system,
            # matching Fusion's "Save as Mesh" while that component is
            # active. Exporting the root-level occurrence would include its
            # assembly placement and rotate/translate the STL.
            component = link['occurrence'].component
            options = export_manager.createSTLExportOptions(
                component, file_name
            )
            options.sendToPrintUtility = False
            options.isBinaryFormat = True
            options.isOneFilePerBody = False
            options.unitType = (
                adsk.fusion.DistanceUnits.MillimeterDistanceUnits
            )
            options.meshRefinement = (
                adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            )
            if not export_manager.execute(options):
                failures.append(link['source_name'])
                continue
            translation, axes = relative_occurrence_transform(
                root_occurrence, link['occurrence']
            )
            transform_binary_stl(file_name, translation, axes)
        except Exception:
            failures.append(link['source_name'])

    if failures:
        raise RuntimeError(
            'Failed to export STL for: {}.'.format(', '.join(failures))
        )


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")
