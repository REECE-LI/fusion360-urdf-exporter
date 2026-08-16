#Author-syuntoku14, Dheena2k2, Lentin Joseph,
#Description-Generate a URDF file and STL meshes from Fusion 360

import os

import adsk
import adsk.core
import adsk.fusion

from .utils import utils
from .core import Link, Joint, Write

"""
# Fusion length units are centimetres and inertia units are kg/cm^2.
# If there is no body in the root component, coordinates may be incorrect.
"""

# supports "Revolute", "Rigid" and "Slider" joint types


def build_success_message(save_dir, robot_name, loop_joints):
    """Build the completion dialog text, including loop-constraint data."""
    message = (
        'Successfully created:\n'
        + os.path.join(save_dir, robot_name + '.urdf')
        + '\n'
        + os.path.join(save_dir, 'meshes')
    )
    if loop_joints:
        closed_chain_names = [
            joint['source_name'] for joint in loop_joints
        ]
        message += (
            '\n\nClosed-chain constraints embedded in URDF:\n'
            + '\n'.join(closed_chain_names)
        )
    return message


def run(context):
    ui = None

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion 360 -> URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component

        robot_name = 'robot'
        # Show welcome message
        welcome_msg = ("Welcome to the Fusion 360 -> URDF exporter.\n"
                       "\n"
                       "This tool generates a portable URDF file and STL meshes from the active Fusion design.\n"
                       "\n"
                       "Press OK to continue or Cancel to quit.")
        if ui.messageBox(welcome_msg, title, adsk.core.MessageBoxButtonTypes.OKCancelButtonType) != adsk.core.DialogResults.DialogOK:
            return

        # Show folder browse message
        browse_msg = "Press OK to choose where to save the robot folder, or Cancel to quit."
        if ui.messageBox(browse_msg, title, adsk.core.MessageBoxButtonTypes.OKCancelButtonType) != adsk.core.DialogResults.DialogOK:
            return

        # Browse folder
        save_dir = utils.file_dialog(ui)
        if not save_dir:
            ui.messageBox('Fusion 360 -> URDF export was canceled', title)
            return 0

        save_dir = os.path.join(save_dir, robot_name)

        # Validate and collapse the design before creating output files.
        model = Joint.build_robot_model(root)
        inertial_dict = Link.make_inertial_dict(model['links'])

        # Export every top-level subassembly as one complete STL without
        # modifying the Fusion design.
        utils.export_stl(
            design,
            save_dir,
            model['links'],
            model['root_occurrence'],
        )
        Write.write_urdf(
            model['links'],
            model['joints'],
            model['loop_joints'],
            inertial_dict,
            robot_name,
            save_dir,
        )

        success_message = build_success_message(
            save_dir, robot_name, model['loop_joints']
        )
        ui.messageBox(success_message, title)

    except Exception as e:
        if ui:
            ui.messageBox(f'Failed:\n{str(e)}', title)
