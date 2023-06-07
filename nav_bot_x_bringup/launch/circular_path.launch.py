import launch
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    # Create a launch description object
    ld = LaunchDescription()

    # Add your launch actions to the launch description
    ld.add_action(launch_ros.actions.Node(
        package='NavBotX',
        executable='emergency_button',
        name='emergency_button'
    ))

    ld.add_action(launch_ros.actions.Node(
        package='NavBotX',
        executable='pub_path',
        name='circular_path'
    ))

    ld.add_action(launch_ros.actions.Node(
        package='NavBotX',
        executable='navigation',
        name='controller'
    ))

    return ld

if __name__ == '__main__':
    # Generate the launch description
    ld = generate_launch_description()

    # Launch the system
    launch.launch(ld)

