import shutil
import os
import random
import xml.etree.ElementTree as ET
from copy import deepcopy

import numpy as np

#### EXTRA IMPORTS
import lxml.etree as le
import trimesh
from drake_conversion.just_geom_conversion import convert_geoms_to_obj
# just_geom_conversion.py

# from drake_conversion.auto_texture import execute
from drake_conversion.add_color import execute
from drake_conversion.remove_cab_doors import rm_cab_doors
from drake_conversion.remove_collision import rm_collision

def convert_relative_to_absolute(xml_file):
    """
    Helper function to convert relative paths to absolute paths

    Input:
        xml_file (string)

    Output:
        string of the xml with absolute paths
    """
    # Get the absolute path of the XML file
    base_path = os.path.dirname(os.path.abspath(xml_file))
    
    # Parse the XML file
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # Function to convert relative paths to absolute paths
    def update_file_attribute(element, attribute_name):
        relative_path = element.get(attribute_name)
        if relative_path:
            # Convert to absolute path
            absolute_path = os.path.abspath(os.path.join(base_path, relative_path))
            # Update the attribute with the absolute path
            element.set(attribute_name, absolute_path)

    # Update <compiler> meshdir attribute
    compiler = root.find('.//compiler')
    if compiler is not None:
        update_file_attribute(compiler, 'meshdir')

    # Iterate through all <texture> and <mesh> elements in the XML
    for texture in root.findall('.//texture'):
        update_file_attribute(texture, 'file')
    
    for mesh in root.findall('.//mesh'):
        update_file_attribute(mesh, 'file')
    
    xml_string = ET.tostring(root, encoding='utf-8', xml_declaration=True).decode('utf-8')
    return xml_string



"""
ADDED: PYDRAKE CONVERSION!!
"""
from datetime import datetime

original_model_path = "../xml120_rel/env001.xml"
xml_filename = original_model_path.split("/")[-1][:-4] + ".drake.xml"

doc = le.fromstring(convert_relative_to_absolute(original_model_path).encode("utf-8"))
for elem in doc.xpath("//*[attribute::name]"):
    if (
        # rm robot
        "robot0" in elem.attrib["name"]
        or "base0" in elem.attrib["name"]
        or "gripper0" in elem.attrib["name"]
        or "omniron" in elem.attrib["name"]
    ):
        parent = elem.getparent()
        parent.remove(elem)

new_foldername = os.path.abspath(f"objs")
if not os.path.exists(new_foldername):
    os.makedirs(new_foldername)

# Recreate the scales
for elem in doc.xpath("//*[attribute::scale]"):
    # parse out the scale
    scal = elem.attrib["scale"].split()
    scal = [float(s) for s in scal]
    # multiscale but not uniform
    if len(scal) == 3 and not (scal[0] == scal[1] and scal[1] == scal[2]):
        # recreate the object with the new scaled object
        file_name = elem.attrib["file"]
        mesh = trimesh.load_mesh(file_name)
        mesh.apply_scale(scal)
        new_filename = os.path.abspath(f"objs/{file_name.split('/')[-1]}")
        print(file_name.split("/")[-1])
        print(new_filename)
        mesh.export(new_filename)
        elem.attrib["file"] = new_filename
        del elem.attrib["scale"]

# open the new file
with open(xml_filename, "w") as f:
    new_xml_str = (le.tostring(doc)).decode("utf-8")
    f.write(new_xml_str)

    # convert and clean
    convert_geoms_to_obj(xml_filename)
    execute(xml_filename)
    # rm_cab_doors(xml_filename)
    rm_collision(xml_filename)

# # small cleanup with the relative filepaths -- need to move the obj file up
# folder_to_move = 'objs'
# new_location = os.path.join('..', folder_to_move)
# shutil.move(folder_to_move, new_location)

# run 

from pydrake.all import ModelVisualizer, PackageMap, Simulator, StartMeshcat
from manipulation import ConfigureParser, FindResource, running_as_notebook
# from manipulation.station import LoadScenario, MakeHardwareStation

# Start the visualizer.
meshcat = StartMeshcat()

visualizer = ModelVisualizer(meshcat=meshcat)
ConfigureParser(visualizer.parser())
visualizer.AddModels(xml_filename)
visualizer.Run(loop_once=0)
meshcat.DeleteAddedControls()

input("Open localhost:7000\nPress enter to quit ")
