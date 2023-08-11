from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Load the moveit_configuration from the 
    # ur5e_cell_moveit_config package and add the
    # moveit_cpp.yaml file from the ur5e_cell_pick_n_place
    # package to the parameters
    moveit_config = (
        MoveItConfigsBuilder("prbt_cell", package_name="prbt_cell_moveit_config")
        .robot_description(file_path=get_package_share_directory("prbt_cell_description") + "/urdf/prbt_cell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("prbt_cell_pick_n_place")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )
    
    # Run the pick_n_place_node with the loaded 
    # moveit_config
    moveit_cpp_node = Node(
        name="pick_n_place_node",
        package="prbt_cell_pick_n_place",
        executable="pick_n_place_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription(
        [
            moveit_cpp_node
        ]
    )