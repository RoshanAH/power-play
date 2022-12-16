package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.roshanah.jerky.profiling.*
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.rad
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlinx.coroutines.CoroutineScope

@TeleOp(group="testing")
@Config
class JerkyTuner : BaseOpmode() {

  var currentProfile: Bounded = { ProfilePoint.zero }
  var constants = DriveConstants(maxVel, maxAccel, trackWidth * 0.5, PSVAConstants(kP, kS, kV, kA))
  var forward = true
  var timeInterval = 0.0
  lateinit var voltageSensor: VoltageSensor
  var volts = 12.0
  lateinit var dt: Mecanum 
  
  enum class Mode {
    DRIVE,
    TUNE;
    val other: Mode
      get() = when (this) {
        DRIVE -> TUNE
        TUNE -> DRIVE
      }
  }

  override fun setRobot() = object : Robot() {
    override fun mapHardware(map: HardwareMap){
      voltageSensor = map.voltageSensor.elementAt(0)

      dt = Mecanum(map, "fl", "fr", "bl", "br") {
        volts
      }.apply{
        ticksPerInch = 30.9861111
        ticksPerDegree = 4.98611
      }

      addComponents(dt)    
    }
  }

  override fun onInit(scope: CoroutineScope){
      telemetry = MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry)
  }

  override fun onStart(scope: CoroutineScope){
    gamepadListener1.a.onPress = {
      mode = mode.other
      if (mode == Mode.TUNE) {
        resetRuntime()
        forward = true
        switchProfile()
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    volts = voltageSensor.getVoltage()

    when (mode) {
      Mode.DRIVE -> {
        dt.drive(gamepad1)
      }
      Mode.TUNE -> {
        val point = currentProfile(runtime)
        dt.move(constants.psva(point.wheels, dt.vel))

        if (runtime > timeInterval) {
          forward = !forward
          resetRuntime()
          switchProfile()
        }

        point.wheels.vel.apply{
          telemetry.addData("tflv", fl)
          telemetry.addData("tflv", fr)
          telemetry.addData("tblv", bl)
          telemetry.addData("tbrv", br)
        }
      }
    }

    dt.vel.apply{
      telemetry.addData("flv", fl)
      telemetry.addData("frv", fr)
      telemetry.addData("blv", bl)
      telemetry.addData("brv", br)
    }
    telemetry.addData("volts", volts)
    telemetry.update()
  }

  private fun switchProfile(){
    constants = DriveConstants(maxVel, maxAccel, trackWidth * 0.5, PSVAConstants(kP, kS, kV, kA))
    val sign = if (forward) 1.0 else -1.0

    val newProfile = buildProfile(constants){
      val speedUp = to(0.0, maxVel * sign, 0.0)
      displace(displacement - (speedUp.displacement.pos * 2.0).magnitude)
      stop()
    }

    currentProfile = newProfile
    timeInterval = newProfile.length
  }
}
