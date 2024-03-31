import math
import omni.kit.commands

import omni
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.kit.viewport.utility
from pxr import UsdGeom
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.kit.viewport.utility import get_viewport_from_window_name
from omni.isaac.sensor import ContactSensor
import numpy as np

prim_path = "/World/Cube"

sensor = ContactSensor(
    prim_path = prim_path + "/Contact_Sensor",
    name = "Contact_Sensor",
    frequency = 30,
    translation = np.array([0, 0, 0]),
    min_threshold = 0.0001,
    max_threshold = 10000000,
    radius = -1
)

keys = og.Controller.Keys
(ros_contact_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": prim_path + "/Contact_Graph",
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("readContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
            ("selectIf", "omni.graph.nodes.SelectIf"),
            ("falseValue", "omni.graph.nodes.ConstantUChar"),
            ("trueValue", "omni.graph.nodes.ConstantUChar"),
            ("makeArray", "omni.graph.nodes.ConstructArray"),
            ("publishSensorValue", "omni.isaac.ros2_bridge.ROS2PublishImage"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "readContactSensor.inputs:execIn"),
            ("readContactSensor.outputs:execOut", "publishSensorValue.inputs:execIn"),
            ("readContactSensor.outputs:inContact", "selectIf.inputs:condition"),
            ("falseValue.inputs:value", "selectIf.inputs:ifFalse"),
            ("trueValue.inputs:value", "selectIf.inputs:ifTrue"),
            ("selectIf.outputs:result", "makeArray.inputs:input0"),
            ("makeArray.outputs:array", "publishSensorValue.inputs:data"),
        ],
        keys.SET_VALUES: [
            ("readContactSensor.inputs:csPrim", prim_path + "/Contact_Sensor"),
            ("falseValue.inputs:value", 0),
            ("trueValue.inputs:value", 255),
            ("makeArray.inputs:arraySize", 3),
            ("publishSensorValue.inputs:height", 1),
            ("publishSensorValue.inputs:width", 1),
        ],
    },
)

og.Controller.evaluate_sync(ros_contact_graph)

