import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.roshanah.jerky.profiling.*
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.rad
import com.roshanah.jerky.math.newtonMethodSolve
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.IMU
import org.firstinspires.ftc.teamcode.opmodes.jerky.JerkyConstants
import org.firstinspires.ftc.teamcode.robots.jawsHubOrientation
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlinx.coroutines.CoroutineScope
import kotlin.math.PI

@TeleOp(group="testing")
@Config
class JerkyTuner : BaseOpmode() {
  //beef jerky tuner
  var currentProfile: Bounded = Mode.FORWARD.profile()
  var tuning = false
  lateinit var voltageSensor: VoltageSensor
  var volts = 12.0
  lateinit var dt: Mecanum 
  lateinit var imu: IMU
  
  enum class Mode(val profile: () -> Bounded) {
    FORWARD({
      buildProfile(JerkyConstants.constants){
        to(0.0, JerkyConstants.maxVel, 0.0)
        stop()
        to(0.0, -JerkyConstants.maxVel, 0.0)
        stop()
      }
    }),
    STRAFE({
      buildProfile(JerkyConstants.constants){
        to(JerkyConstants.maxVel, 0.0, 0.0)
        stop()
        to(-JerkyConstants.maxVel, 0.0, 0.0)
        stop()
      }
    }),
    TURN({
      buildProfile(JerkyConstants.constants){
        val omega = constants.maxVelocity / (2 * constants.trackRadius)
        val toSpeed = to(0.0, 0.0, omega)
        displace((PI - toSpeed.displacement.heading * 2).rad)
        stop()
        to(0.0, 0.0, -omega)
        displace((-PI + toSpeed.displacement.heading * 2).rad)
        stop()
      }
    }),
    MOVE_AND_TURN({
      val constants = JerkyConstants.constants
      val omega = newtonMethodSolve({
          buildProfile(constants){
            to(0.0, JerkyConstants.maxVel * 0.25, it)
            stop()
          }.displacement.heading - PI
        },
        0.0,
        constants.maxVelocity / (2 * constants.trackRadius)
      )

      buildProfile(constants){
        to(0.0, JerkyConstants.maxVel * 0.25, omega)
        stop()
      }
    });
  }

  companion object{
    @JvmField var mode = Mode.FORWARD
  }

  override fun setRobot() = object : Robot() {
    override fun mapHardware(map: HardwareMap){
      voltageSensor = map.voltageSensor.elementAt(0)

      dt = Mecanum(map, "fl", "fr", "bl", "br") {
        volts
      }.apply{
        ticksPerInch = 30.9861111
        trackRadius = JerkyConstants.trackRadius
      }

      imu = IMU(map, "imu", jawsHubOrientation)

      addComponents(dt, imu)    
    }
  }

  override fun onInit(scope: CoroutineScope){
      telemetry = MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry)
  }

  override fun onStart(scope: CoroutineScope){
    gamepadListener1.a.onPress = {
      tuning = !tuning
      resetRuntime()
      if(tuning){
        currentProfile = mode.profile()
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    volts = voltageSensor.getVoltage()

    if (!tuning){
      dt.drive(gamepad1)
    }else {
      if(runtime > currentProfile.length){
        resetRuntime()
        currentProfile = mode.profile()
      }

      val constants = JerkyConstants.constants
      val point = currentProfile(runtime)

      dt.trackRadius = constants.trackRadius

      dt.move(constants.psva(point, dt.relativeVel))

      constants.wheels(point).vel.combine().apply{
        telemetry.addData("tflv", fl)
        telemetry.addData("tfrv", fr)
        telemetry.addData("tblv", bl)
        telemetry.addData("tbrv", br)
      }

      telemetry.addData("tw", point.vel.heading)

      // println(constants.wheels(point).vel.forward)

    }

    dt.vel.apply{
      telemetry.addData("flv", fl)
      telemetry.addData("frv", fr)
      telemetry.addData("blv", bl)
      telemetry.addData("brv", br)
    }
    telemetry.addData("volts", volts)
    telemetry.addData("omega", imu.velocity)
    telemetry.update()
  }
}
