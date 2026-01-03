package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.NewmanAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.RobotTeam;
import org.firstinspires.ftc.teamcode.pathing.PathDirection;
import org.firstinspires.ftc.teamcode.pathing.PathPart;
import org.firstinspires.ftc.teamcode.pathing.PathRotation;

@SuppressWarnings("unused")
@Autonomous(name = "Autonomous: Red, Goal Position", group = "Newman")
public class NewmanAutonomousRedGoal extends NewmanAutonomous {
    @Override public void setup() {
        forward_path = new PathPart[]{
            PathPart.Drive(PathDirection.REVERSE, 2.0)
        };
        reverse_path = new PathPart[]{
            PathPart.Rotate(PathRotation.LEFT, 45.0),
            PathPart.Drive(PathDirection.REVERSE, 6.0)
        };
        robot_team = RobotTeam.RED;
    }
}
