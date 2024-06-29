import launch
import launch.actions



def generate_launch_description():
    # Define the path to your launch file
    launch_file_path = '/home/yomnahashem/Formula24/src/fs-system/supervisor/launch/staticA_launch.py'

    # Include the launch description from your staticA_launch.py file
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path)
        ),
    ])

def main():
    # Create the launch service
    launch_service = launch.LaunchService()

    # Generate the launch description
    launch_description = generate_launch_description()

    # Include the launch description in the launch service
    launch_service.include_launch_description(launch_description)

    # Run the launch service
    launch_service.run()

if __name__ == '__main__':
    main()
