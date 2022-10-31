package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.robots.Jaws

@TeleOp
class DrivetrainTest : BaseOpmode() {
  val robot = Jaws()

  override fun setRobot(): Robot = robot

  override fun onStart() {
    gamepadListener1.onJoystickMove = { robot.drivetrain.drive(gamepad1) }
  }
}
