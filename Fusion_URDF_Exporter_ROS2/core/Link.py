# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:11:28 2019

@author: syuntoku
"""

import adsk
import adsk.fusion
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Link:

    def __init__(self, name, xyz, center_of_mass, mass, inertia_tensor):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        # xyz for visual
        self.xyz = [-_ for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        
    def make_link_xml(self):
        """
        Generate the link_xml and hold it by self.link_xml
        """
        
        link = Element('link')
        link.attrib = {'name':self.name}
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = \
            {'ixx':str(self.inertia_tensor[0]), 'iyy':str(self.inertia_tensor[1]),\
            'izz':str(self.inertia_tensor[2]), 'ixy':str(self.inertia_tensor[3]),\
            'iyz':str(self.inertia_tensor[4]), 'ixz':str(self.inertia_tensor[5])}        
        
        # visual
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {
            'filename': 'meshes/' + self.name + '.stl',
            'scale': '0.001 0.001 0.001'
        }
        material = SubElement(visual, 'material')
        material.attrib = {'name':'silver'}
        
        # collision
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {
            'filename': 'meshes/' + self.name + '.stl',
            'scale': '0.001 0.001 0.001'
        }

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])


def make_inertial_dict(links):
    """Return aggregate physical properties for each top-level link."""
    inertial_dict = {}

    for link in links:
        occurrence = link['occurrence']
        properties = occurrence.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        )
        mass = properties.mass
        center_of_mass = [
            value / 100.0
            for value in properties.centerOfMass.asArray()
        ]

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        (_, xx, yy, zz, xy, yz, xz) = (
            properties.getXYZMomentsOfInertia()
        )
        inertia_at_world_origin = [
            value / 10000.0
            for value in [xx, yy, zz, xy, yz, xz]
        ]
        inertial_dict[link['name']] = {
            'name': link['source_name'],
            'mass': mass,
            'center_of_mass': center_of_mass,
            'inertia': utils.origin2center_of_mass(
                inertia_at_world_origin, center_of_mass, mass
            ),
        }

    return inertial_dict
