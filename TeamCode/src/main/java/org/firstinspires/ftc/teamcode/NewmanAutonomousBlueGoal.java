package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.NewmanAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.RobotTeam;
import org.firstinspires.ftc.teamcode.pathing.PathDirection;
import org.firstinspires.ftc.teamcode.pathing.PathPart;
import org.firstinspires.ftc.teamcode.pathing.PathRotation;

@SuppressWarnings("unused")
@Autonomous(name = "Autonomous: Blue, Goal Position", group = "Newman")
public class NewmanAutonomousBlueGoal extends NewmanAutonomous {
    @Override public void setup() {
        forward_path = new PathPart[]{
            PathPart.Drive(PathDirection.REVERSE, 2.0)
        };
        reverse_path = new PathPart[]{
            PathPart.Drive(PathDirection.REVERSE, 1.2),
            PathPart.Rotate(PathRotation.RIGHT, 45.0),
            PathPart.Drive(PathDirection.REVERSE, 4.0)
        };
        robot_team = RobotTeam.BLUE;
    }
}
