package org.firstinspires.ftc.teamcode.systems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.Pair;
import kotlin.jvm.internal.Intrinsics;


public final class JacobianArm {
    private final double L1;
    private final double L2;
    private final double TICKS_PER_DEGREE;
    private final DcMotorEx dof1;
    private final int dof1_offset;
    private boolean dof1reset;
    private final DcMotorEx dof2;
    private final int dof2_offset;
    private boolean dof2reset;
    private final HardwareMap hardwareMap;
    private boolean moving;

    public JacobianArm(HardwareMap hardwareMap) {
        Intrinsics.checkNotNullParameter(hardwareMap, "hardwareMap");
        this.hardwareMap = hardwareMap;
        Object obj = this.hardwareMap.get((Class<? extends Object>) DcMotorEx.class, "dof1");
        Intrinsics.checkNotNullExpressionValue(obj, "get(...)");
        this.dof1 = (DcMotorEx) obj;
        Object obj2 = this.hardwareMap.get((Class<? extends Object>) DcMotorEx.class, "dof2");
        Intrinsics.checkNotNullExpressionValue(obj2, "get(...)");
        this.dof2 = (DcMotorEx) obj2;
        this.L1 = 43.0d;
        this.L2 = 50.0d;
        this.TICKS_PER_DEGREE = 19.793055555555554d;
        this.dof1_offset = ArmConfig.offset1;
        this.dof2_offset = ArmConfig.offset2;
        this.dof2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.dof1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dof2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dof1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.dof2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetArm(){
        this.dof1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dof2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /* renamed from: goto, reason: not valid java name */
    public final boolean ArmGoto(double x, double y, boolean elbowUp) {
        double rSquared = (x * x) + (y * y);
        double cosTheta2 = ((rSquared - (this.L1 * this.L1)) - (this.L2 * this.L2)) / ((2 * this.L1) * this.L2);
        if (cosTheta2 < -1.0d || cosTheta2 > 1.0d) {
            return false;
        }
        double sinTheta2 = Math.sqrt(1 - (cosTheta2 * cosTheta2));
        if (!elbowUp) {
            sinTheta2 = -sinTheta2;
        }
        double theta2_relative = Math.atan2(sinTheta2, cosTheta2);
        double k1 = this.L1 + (this.L2 * cosTheta2);
        double k2 = this.L2 * sinTheta2;
        double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
        double theta2_absolute = theta1 + theta2_relative;
        double theta1_deg = Math.toDegrees(theta1);
        double theta2_deg = Math.toDegrees(theta2_absolute);
        int target1Ticks = (int) (this.TICKS_PER_DEGREE * theta1_deg);
        int target2Ticks = (int) (this.TICKS_PER_DEGREE * theta2_deg);
        this.dof1.setTargetPosition(this.dof1_offset + target1Ticks);
        this.dof2.setTargetPosition(this.dof2_offset + target2Ticks);
        this.dof1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.dof2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.dof1.setPower(1.0d);
        this.dof2.setPower(1.0d);
        this.moving = true;
        return true;
    }

    public final boolean isMoving() {
        if (Math.abs(dof1.getTargetPosition() - dof1.getCurrentPosition()) > 50 || Math.abs(dof2.getTargetPosition() - dof2.getCurrentPosition()) > 50) {
            return true;
        }
        else {
            return false;
        }
    }

    public Pair<Double, Double> getPosition() {
        double theta1_deg = (this.dof1.getCurrentPosition() + this.dof1_offset) / this.TICKS_PER_DEGREE;
        double theta2_deg = (this.dof2.getCurrentPosition() + this.dof2_offset) / this.TICKS_PER_DEGREE;
        double theta1_rad = Math.toRadians(theta1_deg);
        double theta2_rad = Math.toRadians(theta2_deg);
        double x = (this.L1 * Math.cos(theta1_rad)) + (this.L2 * Math.cos(theta2_rad));
        double y = (this.L1 * Math.sin(theta1_rad)) + (this.L2 * Math.sin(theta2_rad));
        return new Pair<>(Double.valueOf(x), Double.valueOf(y));
    }
}