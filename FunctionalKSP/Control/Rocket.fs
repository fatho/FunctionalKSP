module FunctionalKSP.Control.Rocket

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System

open FunctionalKSP.Extensions
open FunctionalKSP.Math
open FunctionalKSP.Units
open FunctionalKSP.Mechanics
open FunctionalKSP.Telemetry
open KRPC.Client.Services.RemoteTech
open MathNet.Numerics

type SteeringMode =
    | KillRotation
    | LockTarget of vec3<1> * ReferenceFrame
    | LockTargetUp of vec3<1> * vec3<1> * ReferenceFrame

/// Computes perfect steering based on the available torque. Only works well in space.
/// In an atmosphere, aerodynamic forces introduce too much disturbance for the formulas to still hold.
/// TODO: take into account and counteract engine torque
type ZeroGSteering(ksc: SpaceCenter.Service, ship: Vessel) =
    let conn = ship.Connection
    
    // Streams for data about ship mechanics
    // Available torque and moment of inertia change while fuel is consumed
    let availableTorqueS = conn.UseStream<N m>(fun () -> ship.AvailableTorque)
    let moiS = conn.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
    let orbitalAngularVelocityS = conn.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))

    let debug = true

    // debug output
    do
        if debug then
            let torquePos, torqueNeg = availableTorqueS.Value
            let moi = moiS.Value
            let accelPos, accelNeg = Inertia.angularAcceleration moi torquePos, Inertia.angularAcceleration moi torqueNeg

            printfn "torque+ %s" (torquePos.ToString())
            printfn "torque- %s" (torqueNeg.ToString())
            printfn "moi     %s" (moi.ToString())
            printfn "accel+  %s" (accelPos.ToString())
            printfn "accel-  %s" (accelNeg.ToString())

            let avel = Vec3.pack <| ship.AngularVelocity(ship.OrbitalReferenceFrame)
            let axis, speed = Vec3.norm avel, Vec3.mag avel * 1.<rad/s>
            let shipAxis = ksc.TransformDirection(Vec3.unpack axis, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
            printfn "ang.vel. axis  %O  speed %.2f deg/s" axis (speed * degPerRad)
            printfn "ang.vel. ship axis  %O" shipAxis
        
    // mutable state
    let mutable lastUT: float<s> = ksc.UT.As<s>()

    // implementation

    let controlAngularAcceleration (targetAngAccel: vec3<rad/s^2>) =
        // get actual available angular acceleration
        let torquePos, torqueNeg = availableTorqueS.Value
        let moi = moiS.Value
        let accelPos, accelNeg = Inertia.angularAcceleration moi torquePos, Inertia.angularAcceleration moi torqueNeg
        
        // compute control inputs for achieving this angular acceleration
        let pitchCommand = targetAngAccel.x / if targetAngAccel.x > 0.<_> then accelPos.x else - accelNeg.x
        let rollCommand = targetAngAccel.y / if targetAngAccel.y > 0.<_> then accelPos.y else - accelNeg.y
        let yawCommand = targetAngAccel.z / if targetAngAccel.z > 0.<_> then accelPos.z else - accelNeg.z
        
        if debug then printfn "CTRL %.3f %.3f %.3f" pitchCommand rollCommand yawCommand

        // clamp inputs in case we need more acceleration than we have
        let sanitizeInput control = clamp -1. 1. <| if abs control < 0.000001 then 0. else control
        
        ship.Control.Pitch <- sanitizeInput pitchCommand |> float32U
        ship.Control.Roll <- sanitizeInput rollCommand |> float32U
        ship.Control.Yaw <- sanitizeInput yawCommand |> float32U

    let killRotation (dt: float<s>): vec3<rad / s^2> =
        let angvel = -orbitalAngularVelocityS.Value
        let shipAngVel = Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack angvel, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
        
        // Try to stop the angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        let horizon = 1.<s> 

        if debug then printfn "%O %O" shipAngVel (-shipAngVel / horizon)

        // compute the angular acceleration that is necessary to stop within the desired horizon
        -shipAngVel / horizon
    
    let lockTarget (dt: float<s>) (shipTarget: vec3<1>) (shipUp: vec3<1>): vec3<rad / s^2> =
        let shipAngVel = -(Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack orbitalAngularVelocityS.Value, ship.OrbitalReferenceFrame, ship.ReferenceFrame))
        
        let axisAngle v1 v2 =
            let axis = Vec3.cross v1 v2
            let angle = Vec3.angle v1 v2
            if angle > 179.9999999<deg> / degPerRad then
                // vectors are 180 degrees apart, rotate around pitch axis
                Vec3.unitX, angle
            else
                Vec3.norm axis, angle

        // rotation axis for turning towards target
        let rollTarget = axisAngle Vec3.unitZ (- shipUp)

        let pitchYawAxis, pitchYawAngle = axisAngle Vec3.unitY shipTarget
        
        // Try to reach the target angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot
        let horizon = 1.<s>

        if pitchYawAngle > 0.05<deg> / degPerRad then

            // only adjust roll within this deviation of the target orientation
            let rollControlRange = 5.<deg>
            let rollControlScale (deviation: float<deg>) = 1. / (1. + exp ((deviation - rollControlRange) / 1.<deg>))
                        
            // get actual available angular acceleration
            let torquePos, torqueNeg = availableTorqueS.Value
            let moi = moiS.Value
            let accelPos, accelNeg = Inertia.angularAcceleration moi torquePos, Inertia.angularAcceleration moi torqueNeg
        
            // simplify acceleration treatment. Use slowest of both directions
            let availableAcceleration = Vec3.zip min accelPos -accelNeg
        
            // rotational component along target axis
            let existingPitchYawAngVel = Vec3.project shipAngVel pitchYawAxis
            // rotational component that we don't want
            let excessAngVel = shipAngVel - existingPitchYawAngVel
            // compute available acceleration towards target
            let pitchYawAccel = Vec3.project availableAcceleration pitchYawAxis

            // how long does it take to reduce angular velocity to zero at max acceleration
            let fullStopTime = Vec3.mag existingPitchYawAngVel / Vec3.mag pitchYawAccel
                
            let fullStopDistance = 0.5 * Vec3.mag existingPitchYawAngVel * fullStopTime
        
        
            printfn "%.2f %O %.1f %.1f" horizon pitchYawAxis (pitchYawAngle * degPerRad) (Vec3.mag existingPitchYawAngVel * degPerRad)

            // acceleration required for killing excess rotation
            let killExcessAccel = excessAngVel / horizon

            let desiredAccel = 
                if pitchYawAngle < 0.0001<deg> / degPerRad then
                    -existingPitchYawAngVel / horizon
                else
                    // at which initial speed can we stop right on target
                    let optimalSpeed = sqrt (2./3. * pitchYawAngle * Vec3.mag pitchYawAccel * 0.9)

                    let speedError = optimalSpeed - Vec3.mag existingPitchYawAngVel
                    let optimalAccel = pitchYawAxis * speedError / horizon

                    if debug then printfn "S %.3f AV %.3f E %.2f" fullStopTime (optimalSpeed * degPerRad) (speedError * degPerRad)
                
                    -optimalAccel
        
            - killExcessAccel + desiredAccel

        else
            -shipAngVel / horizon


    member val Mode = SteeringMode.KillRotation with get, set

    /// The vessel controlled by this steering instance
    member this.Vessel with get(): Vessel = ship

    member this.OrbitalAngularVelocity with get(): vec3<deg/s> = orbitalAngularVelocityS.Value * degPerRad

    member this.Update(ut: float<s>): unit =
        if ut > lastUT then 
            let dt = ut - lastUT
            lastUT <- ut
            let angularAcceleration =
                match this.Mode with
                | KillRotation -> killRotation dt
                | LockTarget (target, ref) -> 
                    let shipTarget = Vec3.pack <| ksc.TransformDirection(Vec3.unpack target, ref, ship.ReferenceFrame)
                    let shipUp = - Vec3.unitZ // Z axis points down in ship reference frame
                    lockTarget dt shipTarget shipUp
                | LockTargetUp (target, up, ref) ->
                    let shipTarget = Vec3.pack <| ksc.TransformDirection(Vec3.unpack target, ref, ship.ReferenceFrame)
                    let shipUp = Vec3.pack <| ksc.TransformDirection(Vec3.unpack up, ref, ship.ReferenceFrame)
                    lockTarget dt shipTarget shipUp
            controlAngularAcceleration angularAcceleration

    interface IDisposable with
        member this.Dispose() =
            (availableTorqueS :> IDisposable).Dispose()
            (moiS :> IDisposable).Dispose()
            (orbitalAngularVelocityS :> IDisposable).Dispose()


/// Steering input in ship reference frame
type AtmosphericSteeringInput = {
    sampleTime: float<s>;
    targetForward: vec3<1>;
    targetTop: vec3<1>;
    controlTorque: vec3<N m>;
    momentOfInertia: vec3<kg m^2>;
    angularVelocity: vec3<rad/s>
}


type TorquePI() =
    member val Loop = new PidLoop<rad/s, N m>(1.<_>, 0.<_>, 0.<_>) with get
    
    member val Ts: float<s> = 2.<s> with get, set

    member this.Update(sampleTime: float<s>, input: float<rad/s>, setpoint: float<rad/s>, momentOfInertia: float<kg m^2>, maxOutput: float<N m>) =
        let sqr x = x * x

        this.Loop.Ki <- momentOfInertia * sqr (4. / this.Ts) / 1.<rad>
        this.Loop.Kp <- 2. * momentOfInertia * 4. / this.Ts / 1.<rad>
        this.Loop.Update(sampleTime, input, setpoint, maxOutput)

/// Rocket steering in Atmosphere, based on the very well working kOS SteeringManager
/// https://github.com/KSP-KOS/KOS/blob/develop/src/kOS/Control/SteeringManager.cs
type KosSteering() =
    /// when to set steering input to zero
    let EPSILON = 1e-16

    member val PitchRateLoop = new PidLoop<rad, rad/s>(1.<_>, 0.1<_>, 0.)
    member val YawRateLoop = new PidLoop<rad, rad/s>(1.<_>, 0.1<_>, 0.)
    member val RollRateLoop = new PidLoop<rad, rad/s>(1.<_>, 0.1<_>, 0.)
    member val PitchTorqueLoop = new TorquePI()
    member val YawTorqueLoop = new TorquePI()
    member val RollTorqueLoop = new TorquePI()

    member val MaxStoppingTime = 2.<s> with get, set
    member val RollControlRange = 5.<deg> with get, set

    member val LastPitchInput = 0. with get, set
    member val LastYawInput = 0. with get, set
    member val LastRollInput = 0. with get, set

    member this.UpdatePrediction(input: AtmosphericSteeringInput): vec3<1> =
        let targetForward = input.targetForward
        let targetTop = input.targetTop
        let vesselForward = Vec3.unitY
        let vesselStarboard = Vec3.unitX
        let vesselTop = - Vec3.unitZ

        let phi =
            Vec3.angle vesselForward targetForward * (
                if Vec3.angle vesselTop targetForward > 90.<deg> / degPerRad then -1. else 1.)

        let targetForwardNoStarboard = Vec3.exclude targetForward vesselStarboard
        let phiPitch =
            Vec3.angle vesselForward targetForwardNoStarboard * (
                if Vec3.angle vesselTop targetForwardNoStarboard > 90.<deg> / degPerRad then -1. else 1.)

        let targetForwardNoTop = Vec3.exclude targetForward vesselTop
        let phiYaw =
            Vec3.angle vesselForward targetForwardNoTop * (
                if Vec3.angle vesselStarboard targetForwardNoTop > 90.<deg> / degPerRad then -1. else 1.)
                
        let targetTopNoForward = Vec3.exclude targetTop vesselForward
        let phiRoll =
            Vec3.angle vesselTop targetTopNoForward * (
                if Vec3.angle vesselStarboard targetTopNoForward > 90.<deg> / degPerRad then -1. else 1.)
        
        let maxOmega = this.MaxStoppingTime * Inertia.angularAcceleration input.momentOfInertia input.controlTorque

        this.PitchRateLoop.ExtraUnwind <- true
        this.YawRateLoop.ExtraUnwind <- true
        this.RollRateLoop.ExtraUnwind <- true

        let tgtPitchOmega = this.PitchRateLoop.Update(input.sampleTime, -phiPitch, 0.<_>, maxOmega.x)
        let tgtYawOmega = this.YawRateLoop.Update(input.sampleTime, -phiYaw, 0.<_>, maxOmega.z)
        let tgtRollOmega =
            if abs phi > this.RollControlRange / degPerRad then
                this.RollRateLoop.ResetI()
                0.<rad/s>
            else
                this.RollRateLoop.Update(input.sampleTime, -phiRoll, 0.<_>, maxOmega.y)

        // NOTE: coordinate system is apaprently different from kOS, so this code requires some
        // random negations.
        // TODO: find out why
        let tgtPitchTorque = this.PitchTorqueLoop.Update(input.sampleTime, input.angularVelocity.x,
                                tgtPitchOmega, input.momentOfInertia.x, input.controlTorque.x)
        let tgtYawTorque = this.YawTorqueLoop.Update(input.sampleTime, input.angularVelocity.z,
                                tgtYawOmega, input.momentOfInertia.z, input.controlTorque.z)
        let tgtRollTorque = this.RollTorqueLoop.Update(input.sampleTime, input.angularVelocity.y,
                                tgtRollOmega, input.momentOfInertia.y, input.controlTorque.y)
                                
        let snapToEpsilon v = if abs v < EPSILON then 0. else v

        let clampPitch = 2. * max (abs this.LastPitchInput) 0.05
        this.LastPitchInput <- snapToEpsilon <| clamp -clampPitch clampPitch (tgtPitchTorque / input.controlTorque.x)

        let clampYaw = 2. * max (abs this.LastYawInput) 0.05
        this.LastYawInput <- snapToEpsilon <| clamp -clampYaw clampYaw (tgtYawTorque / input.controlTorque.z)
        
        let clampRoll = 2. * max (abs this.LastRollInput) 0.05
        this.LastRollInput <- snapToEpsilon <| clamp -clampRoll clampRoll (tgtRollTorque / input.controlTorque.y)

        { x = this.LastPitchInput; y = this.LastRollInput; z = this.LastYawInput }
        

    
        
