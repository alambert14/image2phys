# Drake imports
import os
import cv2
import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph, ContactModel, DiagramBuilder,
    LoadModelDirectives, MeshcatVisualizerCpp,
    Parser, ProcessModelDirectives, Simulator
)
import pydrake.common

from manipulation.scenarios import AddRgbdSensors
from manipulation.utils import AddPackagePaths
from manipulation.meshcat_cpp_utils import StartMeshcat, AddMeshcatTriad

from ycb_mass import ycb_mass
import dataset_utils as utils

models_dir = os.path.join(os.path.dirname(__file__), 'models')
object_dir = '/home/aalamber/ycb'


def add_package_paths_local(parser: Parser):
    parser.package_map().Add(
        "drake_manipulation_models",
        os.path.join(pydrake.common.GetDrakePath(),
                     "manipulation/models"))

    parser.package_map().Add("local", models_dir)


def render_system_with_graphviz(diagram, output_file="system_view.gz"):
    """ Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. Borrowed from @pang-tao"""
    from graphviz import Source
    string = diagram.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)


def make_environment():
    directive = os.path.join(models_dir, 'bin_and_cameras.yaml')

    builder = DiagramBuilder()
    meshcat = StartMeshcat()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=2e-4)
    MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)
    parser = Parser(plant)
    AddPackagePaths(parser)
    add_package_paths_local(parser)  # local.
    ProcessModelDirectives(LoadModelDirectives(directive), plant, parser)

    obj_name = np.random.choice(list(ycb_mass.keys()))
    # for obj_name in ycb_mass:
    #     utils.generate_obj_sdf(obj_name)
    #     try:
    #         model = parser.AddModelFromFile(
    #             os.path.join(object_dir, obj_name, 'poisson/model.sdf'))
    #     except:
    #         print('Could not find file')
    #         continue

    model = parser.AddModelFromFile(
        os.path.join(object_dir, obj_name, 'poisson/model.sdf'))

    plant.Finalize()
    AddRgbdSensors(builder, plant, scene_graph)

    diagram = builder.Build()

    context = diagram.CreateDefaultContext()

    simulator = Simulator(diagram, context)

    simulator.AdvanceTo(5.0)
    simulator.set_target_realtime_rate(0.)

    render_system_with_graphviz(diagram)
    return diagram, plant, context, simulator


if __name__ == '__main__':
    diagram, plant, context, simulator = make_environment()

    cam = diagram.GetSubsystemByName('camera')
    cam_context = cam.GetMyMutableContextFromRoot(context)
    img = cam.GetOutputPort('color_image').Eval(cam_context).data

    simulator.AdvanceTo(10)

    img_rgb = img[:, :, :-1]
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    cv2.imwrite('rgb_image.png', img_rgb)
