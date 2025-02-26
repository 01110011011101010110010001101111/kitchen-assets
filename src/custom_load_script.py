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




"""
ADDED: PYDRAKE CONVERSION!!
"""
from datetime import datetime

original_model_path = "../xml120_rel/env000.xml"
xml_filename = original_model_path.split("/")[-1][:-4] + ".drake.xml"
with open(original_model_path, "r") as f1:
    doc = le.fromstring(f1.read())
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

with open(xml_filename, "w") as f:
    new_xml_str = (le.tostring(doc)).decode("utf-8")
    f.write(new_xml_str)

    convert_geoms_to_obj(xml_filename)
    execute(xml_filename)
    # rm_cab_doors(xml_filename)
    rm_collision(xml_filename)


# run 

from pydrake.all import ModelVisualizer, PackageMap, Simulator, StartMeshcat
from manipulation import ConfigureParser, FindResource, running_as_notebook
from manipulation.station import LoadScenario, MakeHardwareStation

# Start the visualizer.
meshcat = StartMeshcat()

visualizer = ModelVisualizer(meshcat=meshcat)
ConfigureParser(visualizer.parser())
visualizer.AddModels(xml_filename)
visualizer.Run(loop_once=0)
meshcat.DeleteAddedControls()

input("Open localhost:7000\nPress enter to quit ")
