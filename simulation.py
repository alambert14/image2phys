# Drake imports

def make_environment():
    directive = ''

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=2e-4)
    parser = Parser(plant)
    AddPackagePaths(parser)
    add_package_paths_local(parser)
    ProcessModelDirectives(LoadModelDirectives(directive), plant, parser)