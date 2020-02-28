package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Rectangle;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

public class Robot extends Rectangle {
    public Coord position;
    public SampleMecanumDriveREV drive;
    public HardwareMap hardwareMap;
    private Field field;

    DcMotor gulperMotor1;
    DcMotor gulperMotor2;
    int gulperPower = 1;

    RobotSide deposit;
    RobotSide cam;
    RobotSide grab;
    RobotSide none;
    public RobotSide[] sides;

    public static double LEN = 18.0;
    public static double HALF_L = LEN / 2; // half the width/length of the robot


    public static enum SIDE {
        CAM, GRAB, DEPOSIT, EMPTY
    }

    public Robot(HardwareMap hardwareMap, Field field, Coord startPosition) {
        super(Rectangle.genCoords(startPosition, LEN));
        this.setSides();
        this.drive = new SampleMecanumDriveREV(hardwareMap);
        this.field = field;

    }

    public void initDevices() {
        gulperMotor1 = hardwareMap.dcMotor.get("intake1");
        gulperMotor2 = hardwareMap.dcMotor.get("intake2");
        gulperMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gulperMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gulperMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gulperMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopGulper();
    }

    public void setSides() {
        LineSegment[] edges = this.getEdges();
        RobotSide deposit = new RobotSide(edges[0], SIDE.DEPOSIT, 180);
        RobotSide cam = new RobotSide(edges[1], SIDE.CAM, 90);
        RobotSide grab = new RobotSide(edges[2], SIDE.GRAB, 0);
        RobotSide none = new RobotSide(edges[3], SIDE.EMPTY, 270);
        sides = new RobotSide[]{deposit, cam, grab, none};
    }

    public RobotSide getRobotSide(Robot.SIDE side) {
        for (RobotSide s : sides) {
            if (s.type == side) {
                return s;
            }
        }
        return null;
    }

    public SampleMecanumDriveREV getDrive() {
        return (SampleMecanumDriveREV) this.drive;
    }

    public void updatePosition(Coord newCoord) {
        this.position = newCoord;
    }

    public void updatePosition(double xToAdd, double yToAdd) {
        this.position.set_x(this.position.getX() + xToAdd);
        this.position.set_y(this.position.getY() + yToAdd);
    }

    public void gulpBlock() {
        double forwardSuckDist = 2.0;
        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(new Pose2d(), new DriveConstraints(10, 10, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0));
        Trajectory trajectory = trajectoryBuilder.forward(forwardSuckDist).build();
        runGulper();
        drive.followTrajectorySync(trajectory);
        stopGulper();
    }
    public void runGulper() {
        gulperMotor1.setPower(gulperPower);
        gulperMotor2.setPower(gulperPower);
    }

    public void ejectBlock() {
        gulperMotor1.setPower(-1 * gulperPower);
        gulperMotor2.setPower(-1 * gulperPower);
    }

    public void stopGulper() {
        gulperMotor1.setPower(0);
        gulperMotor2.setPower(0);
    }


    public void mvToBlock(int index) {
        Block bl = field.blocks.get(index);
        Coord midptBack = bl.getMidptOfBack();
        Coord nextToBlockCoord = new Coord(bl.getXCoordOfSide() + Robot.HALF_L, midptBack.getY() + Robot.HALF_L);
        double distShift = nextToBlockCoord.xDist(midptBack);

        Coord ptBehindBlock = new Coord(nextToBlockCoord.getX(), midptBack.getY() - distShift);
        Trajectory trajectory = drive.trajectoryBuilder().strafeLeft(distShift).build();
        drive.followTrajectorySync(trajectory);
        if (index == 0) {
            field.shiftBlocks(new int[]{0}, distShift);
        } else {
            field.shiftBlocks(new int[]{index - 1, index}, distShift);
        }
    }

    public void mvNextToBlock(int index) {
        // Moves robot right next to the block
        Block bl = field.blocks.get(index);
        Coord midptBack = bl.getMidptOfBack();
        Coord nextToBlockCoord = new Coord(bl.getXCoordOfSide() + Robot.HALF_L, midptBack.getY() + Robot.HALF_L);
        Trajectory trajectory = drive.trajectoryBuilder().splineTo(
                new Pose2d(nextToBlockCoord.getX(), midptBack.getY(), Math.toRadians(0))).build();
        drive.followTrajectorySync(trajectory);
    }

    public void moveOutOfBlockArea(int index) {
        Block bl = field.blocks.get(index);
        Coord midptBack = bl.getMidptOfBack();
        Coord nextToBlockCoord = new Coord(bl.getXCoordOfSide() + Robot.HALF_L, midptBack.getY() + Robot.HALF_L);
        double distShift = nextToBlockCoord.xDist(midptBack);

        Trajectory trajectory = drive.trajectoryBuilder().strafeRight(distShift).build();
        drive.followTrajectorySync(trajectory);
    }

    public void moveToBridgePos() {
        Trajectory trajectory = drive.trajectoryBuilder().splineTo(new Pose2d(Field.parkPosition.getX(), Field.parkPosition.getY(), 0)).build();
        drive.followTrajectorySync(trajectory);
    }

}
