package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

@TeleOp(group = "testing")
@Disabled
class IMUTest : LinearOpMode() {
  override fun runOpMode(){
    val imu = hardwareMap.get(IMU::class.java, "imu")
    val params = IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP))
    imu.initialize(params)

    waitForStart()

    while(opModeIsActive()){

      val orientation = imu.robotYawPitchRollAngles

      telemetry.addData("yaw", orientation.getYaw(AngleUnit.RADIANS))
      telemetry.addData("pitch", orientation.getPitch(AngleUnit.RADIANS))
      telemetry.addData("roll", orientation.getRoll(AngleUnit.RADIANS))
      telemetry.update()
    }
  }
}
