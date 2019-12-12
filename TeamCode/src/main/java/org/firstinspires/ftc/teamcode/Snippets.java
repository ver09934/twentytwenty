public class Snippets {

    // ----- Auton Stuff -----

    /*
    public void turnDegrees(double turnPower, double angleDelta) {

        assert Math.abs(angleDelta) <= 180;

        double startAngle = getIMUAngleConverted();

        double targetAngle = startAngle + angleDelta;

        if (targetAngle > 360) {
            targetAngle = targetAngle % 360;
        }
        else if (targetAngle < 0) {
            targetAngle = targetAngle % 360;
            targetAngle += 360;
        }

        turnPower = Math.copySign(turnPower, angleDelta);

        lfMotor.setPower(turnPower);
        rfMotor.setPower(turnPower);
        lbMotor.setPower(turnPower);
        rbMotor.setPower(turnPower);

        double currentAngle = startAngle;

        if (angleDelta > 0 && targetAngle < startAngle) {
            while (currentAngle >= startAngle || currentAngle < targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta < 0 && targetAngle > startAngle) {
            while (currentAngle <= startAngle || currentAngle > targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta > 0) {
            while (currentAngle < targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else {
            while (currentAngle > targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }

        setAllMotorPowers(0);
    }
    */

    // ElapsedTime rampupTimer = new ElapsedTime();
    // double currentTime = rampupTimer.time(TimeUnit.SECONDS);

    // frontDistanceSensor.getDistance(distanceUnit);

    // ----- Teleop Stuff -----

    /*
    // --- Right Trigger: Build Plate Clamper Servos ---
    if (this.gamepad2.right_trigger > 0.5) {
        if (!gamepad2RightTriggerToggleLock) {
            gamepad2RightTriggerToggleLock = true;
            plateServosUp = !plateServosUp;
        }
    }
    else {
        gamepad2RightTriggerToggleLock = false;
    }
    if (plateServosUp) {
        plateServoLeft.setPosition(plateServoLeftUp);
        plateServoRight.setPosition(plateServoRightUp);
    }
    else {
        plateServoLeft.setPosition(plateServoLeftDown);
        plateServoRight.setPosition(plateServoRightDown);
    }
    telemetry.addData("Build plate servos up", plateServosUp);
    */

    /*
    // --- Y Button: Toggle Winch Power (for safety) ---
    if (this.gamepad2.y) {
        if (!gamepad2YToggleLock) {
            gamepad2YToggleLock = true;
            winchesPowered = !winchesPowered;
        }
    }
    else {
        gamepad2YToggleLock = false;
    }
    if (winchesPowered) {
        winchMotor1.setPower(winchMotor1Power);
        winchMotor2.setPower(winchMotor2Power);
    }
    else {
        winchMotor1.setPower(0);
        winchMotor2.setPower(0);
    }
    telemetry.addData("Winches Powered", winchesPowered);
    */

    /* The encoder functions available to us:
    winchMotor1.setPower(0);
    double tmp = winchMotor1.getPower();
    winchMotor1.setTargetPosition(0);
    int tmp = winchMotor1.getTargetPosition();
    boolean tmp = winchMotor1.isBusy();
    int tmp = winchMotor1.getCurrentPosition();
    */

}
