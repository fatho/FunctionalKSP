module FunctionalKSP.Control.Rocket

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System

open FunctionalKSP.Extensions
open FunctionalKSP.LinearAlgebra
open FunctionalKSP.Units
open FunctionalKSP.Mechanics
open FunctionalKSP.Telemetry
open KRPC.Client.Services.RemoteTech
open MathNet.Numerics

type SteeringMode = KillRotation | LockTarget of vec3<1> * ReferenceFrame

type Steering(ksc: SpaceCenter.Service, ship: Vessel) =
    let conn = ship.Connection
    
    // Streams for data about ship mechanics
    // Available torque and moment of inertia change while fuel is consumed
    let availableTorqueS = conn.UseStream<N m>(fun () -> ship.AvailableTorque)
    let moiS = conn.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
    let orbitalAngularVelocityS = conn.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))

    let debug = false

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
        let sanitizeInput control = clamp -1. 1. <| if abs control < 0.001 then 0. else control
        
        ship.Control.Pitch <- sanitizeInput pitchCommand |> float32U
        ship.Control.Roll <- sanitizeInput rollCommand |> float32U
        ship.Control.Yaw <- sanitizeInput yawCommand |> float32U

    let killRotation (dt: float<s>): vec3<rad / s^2> =
        let angvel = orbitalAngularVelocityS.Value
        let shipAngVel = Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack angvel, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
        
        // Try to stop the angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        let horizon = 5. * dt

        // compute the angular acceleration that is necessary to stop within the desired horizon
        shipAngVel / horizon
    
    let lockTarget (dt: float<s>) (shipTarget: vec3<1>): vec3<rad / s^2> =
        let shipAngVel = Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack orbitalAngularVelocityS.Value, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
        
        // rotation axis for turning towards target
        let targetAxis, targetAngle = Rotation.axisAngle Vec3.unitY shipTarget
        
        // get actual available angular acceleration
        let torquePos, torqueNeg = availableTorqueS.Value
        let moi = moiS.Value
        let accelPos, accelNeg = Inertia.angularAcceleration moi torquePos, Inertia.angularAcceleration moi torqueNeg

        // simplify acceleration treatment. Use slowest of both directions
        let accel = Vec3.zip (fun pos neg -> min pos (-neg)) accelPos accelNeg
        
        // rotational component along target axis
        let existingTargetAngVel = Vec3.project shipAngVel targetAxis
        // rotational component that we don't want
        let excessRotation = shipAngVel - existingTargetAngVel
        // compute available acceleration towards target
        let towardsTargetAccel = Vec3.project accel targetAxis * (targetAngle / (180.<deg> / degPerRad))

        // how long does it take to reduce angular velocity to zero at max acceleration
        let fullStopTime = Vec3.mag existingTargetAngVel / Vec3.mag towardsTargetAccel

        // how long does it take to reduce angular velocity to zero and stop on target at max acceleration
        let onTargetStoppedTime = 2. / 3. * targetAngle / Vec3.mag existingTargetAngVel
        
        let fullStopDistance = 0.5 * Vec3.magSq existingTargetAngVel / Vec3.mag towardsTargetAccel
        
        // Try to reach the target angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        let horizon = 4. * dt
        
        // acceleration required for killing excess rotation
        let killExcessAccel = excessRotation / horizon

        let desiredAccel = 
            if fullStopDistance > max (30.<deg> / degPerRad) (targetAngle * 1.5) then
                /// if we cannot come to a full stop without overshooting more than 50%, just kill all velocity first
                if debug then printfn "TOO FAST!!!"
                existingTargetAngVel / horizon
            else
                // at which initial speed is fullStopTime == onTargetStoppedTime
                let optimalSpeed = sqrt (2./3. * targetAngle * Vec3.mag towardsTargetAccel)

                if debug then printfn "STOP %.3f %.3f ANGVEL %.3f" onTargetStoppedTime fullStopTime optimalSpeed
                
                let speedError = optimalSpeed - Vec3.mag existingTargetAngVel
                let optimalAccel = - targetAxis * speedError / horizon
                optimalAccel
        
        killExcessAccel + desiredAccel

    member val Mode = SteeringMode.KillRotation with get, set

    /// The vessel controlled by this steering instance
    member this.Vessel with get(): Vessel = ship

    member this.Update(ut: float<s>): unit =
        if ut > lastUT then 
            let dt = ut - lastUT
            lastUT <- ut
            let angularAcceleration =
                match this.Mode with
                | KillRotation -> killRotation dt
                | LockTarget (target, ref) -> lockTarget dt (Vec3.pack <| ksc.TransformDirection(Vec3.unpack target, ref, ship.ReferenceFrame))
            controlAngularAcceleration angularAcceleration

    interface IDisposable with
        member this.Dispose() =
            (availableTorqueS :> IDisposable).Dispose()
            (moiS :> IDisposable).Dispose()
            (orbitalAngularVelocityS :> IDisposable).Dispose()
