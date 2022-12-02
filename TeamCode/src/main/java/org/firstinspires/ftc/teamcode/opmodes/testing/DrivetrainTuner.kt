package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.FtcDashboard
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.profiling.buildProfile
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

@TeleOp(group="testing")
@Config
class DrivetrainTuner : BaseOpmode() {

  companion object{
    @JvmField var ks = 0.0
    @JvmField var kv = 0.0
    @JvmField var ka = 0.0
    @JvmField var a = 0.0
    @JvmField var v = 0.0
  }

  private lateinit var dt: Mecanum
  private var vels = DriveValues.zero

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          dt = Mecanum(map, "fl", "fr", "bl", "br").apply {
            ticksPerInch = 30.9861111
            ticksPerDegree = 4.98611
            trackWidth = 9.528
          }
          addComponents(dt)
        }
      }

  override fun onInit(scope: CoroutineScope){
    telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
  }

  override fun onStart(scope: CoroutineScope) {}

  override fun onUpdate(scope: CoroutineScope) {

    val target = Pose(
      gamepad1.left_stick_x.toDouble(), 
     -gamepad1.left_stick_y.toDouble(), 
     -gamepad1.right_stick_x.toDouble() * dt.trackWidth
    )

    val constants = DriveConstants(v, a, dt.trackWidth, PSVAConstants(0.0, ks, kv, ka))

    // targetVel(target, constants)
    dt.drive(gamepad1)

    telemetry.addData("flt", vels.fl)
    telemetry.addData("flv", dt.vel.fl)
    telemetry.addData("target", target)
    telemetry.update()
  }

  fun targetVel(vel: Pose, constants: DriveConstants){
    val profile = buildProfile(constants, dt.relativeVel) { to(vel) }
    vels = profile.start.wheels.vel
    dt.move(constants.sva(profile.start.wheels))
  }
}
