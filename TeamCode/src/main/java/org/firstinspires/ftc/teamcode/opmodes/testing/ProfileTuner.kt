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
class ProfileTuner : BaseOpmode() {

  companion object {
    @JvmField var trackWidth = 9.5
    @JvmField var kP = 0.0
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var displacement = 35.0
    @JvmField var maxAccel = 35.0
    @JvmField var maxVel = 35.0
    @JvmField var mode = Mode.DRIVE
  }


  var currentProfile: (Double) -> ProfilePoint = { ProfilePoint.zero }
  var constants = DriveConstants(maxVel, maxAccel, trackWidth * 0.5, PSVAConstants(kP, kS, kV, kA))
  var forward = true
  var timeInterval = 0.0
  // lateinit var voltageSensor: VoltageSensor
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
      dt = Mecanum(map, "fl", "fr", "bl", "br") {
        volts
      }.apply{
        ticksPerInch = 30.9861111
        ticksPerDegree = 4.98611
      }
      // voltageSensor = map.voltageSensor.get("fl")

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
    // volts = voltageSensor.getVoltage()

    when (mode) {
      Mode.DRIVE -> {
        dt.drive(gamepad1)
      }
      Mode.TUNE -> {
        val point = currentProfile(runtime)
        dt.move(constants.sva(point.wheels))

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

    dt.pos.apply{
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

    val newProfile = constants.run {
      val speedUp = interpolateVelocities(Pose.zero, Pose(0.0, maxVel * sign, 0.0))
      val slowDown = interpolateVelocities(speedUp.end.motion.vel, Pose.zero)

      val interpolationDisplacement = (speedUp.end.motion.pos.pos * 2.0).magnitude

      val maintainDisplacement = displacement(speedUp.end.motion.vel, displacement - interpolationDisplacement)

      speedUp + maintainDisplacement + slowDown
    }

    currentProfile = newProfile
    timeInterval = newProfile.length
  }
}
