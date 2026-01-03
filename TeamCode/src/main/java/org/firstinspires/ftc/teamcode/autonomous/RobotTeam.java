package org.firstinspires.ftc.teamcode.autonomous;

public enum RobotTeam {
    BLUE(20),
    RED(24);

    public int tag_id() {
        return tag_id;
    }

    private final int tag_id;

    RobotTeam(int tag_id) {
        this.tag_id = tag_id;
    }
}
