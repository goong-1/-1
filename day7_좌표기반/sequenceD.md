```mermaid
sequenceDiagram
    participant Cam as Camera
    participant VS as VisionSystem
    participant APP as App (gen_frames)
    participant RM as RobotManager
    participant R1 as Robot 1 (Follower)

    loop Every 0.05s
        Cam->>APP: Capture Frame
        APP->>VS: process_frame(frame)
        VS-->>APP: Marker Positions (10, 11, 20, 21)
        APP->>VS: get_robot_pose()
        VS-->>APP: Center, Distance, Angle
        
        Note over APP: if dist > 23cm & not Manual
        
        APP->>RM: send_command(ID:1, "a+75,d+85")
        RM->>R1: TCP Send (Binary String)
        R1->>RM: Encoder Feedback (L/R)
        RM->>APP: Socket.io Emit ('encoder_data')
    end
