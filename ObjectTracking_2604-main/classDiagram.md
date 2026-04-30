```mermaid
classDiagram
    class VisionSystem {
        +pixel_to_cm: float
        +process_frame(frame)
        +get_robot_pose(markers, left_id, right_id)
    }

    class RobotManager {
        +robots: dict
        +ROBOT_MACS: dict
        +get_mac_address(ip)
        +start_server(socketio)
        +send_command(robot_id, cmd)
        -_handle_robot(client, robot_id)
    }

    class App_Py {
        <<Main Entry>>
        +current_vision_mode: String
        +is_robot1_manual: Boolean
        +gen_frames()
        +handle_emergency_robot1()
    }

    App_Py --> VisionSystem : Uses
    App_Py --> RobotManager : Controls