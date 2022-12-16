package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.FtcDashboard
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.utils.PSVAController
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.rad
import com.roshanah.jerky.profiling.buildProfile
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

@TeleOp(group="testing")
@Config
@Disabled
class DrivetrainTuner : BaseOpmode() {

  companion object{
    @JvmField var ks = 0.0
    @JvmField var kv = 0.0
    @JvmField var ka = 0.0
    @JvmField var a = 0.0
    @JvmField var v = 0.0
  }

  private lateinit var dt: Mecanum
  private lateinit var voltageSensor: HardwareMap.DeviceMapping<VoltageSensor>
  private var vels = DriveValues.zero
  private lateinit var controller: PSVAController

  val voltage: Double
    get() = voltageSensor.elementAt(0).getVoltage()

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          voltageSensor = map.voltageSensor
          dt = Mecanum(map, "fl", "fr", "bl", "br"){
            voltage
          }.apply {
            ticksPerInch = 30.9861111
            ticksPerDegree = 4.98611
            trackWidth = 9.528
          }
          controller = PSVAController(DriveConstants(v, a, dt.trackWidth * 0.5, PSVAConstants(0.0, ks, kv, ka)))
          addComponents(dt)
        }
      }

  override fun onInit(scope: CoroutineScope){
    telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
  }

  override fun onStart(scope: CoroutineScope) {}

  override fun onUpdate(scope: CoroutineScope) {

    val constants = DriveConstants(v, a, dt.trackWidth * 0.5, PSVAConstants(0.0, ks, kv, ka))
    controller.constants = constants

    // controller.update(gamepad1)

    dt.move(constants.psva(controller.wheels, dt.vel))


    telemetry.addData("flt", controller.wheels.vel.fl)
    telemetry.addData("flv", dt.vel.fl)
    telemetry.addData("measured", dt.relativeVel)
    telemetry.addData("vels", controller.vel)
    telemetry.addData("w", dt.relativeVel.heading.rad.deg)
    telemetry.update()
  }

}
