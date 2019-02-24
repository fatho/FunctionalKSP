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

    // debug output
    do
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
        
        printfn "CTRL %.3f %.3f %.3f" pitchCommand rollCommand yawCommand

        // clamp inputs in case we need more acceleration than we have
        let sanitizeInput control = clamp -1. 1. <| if abs control < 0.001 then 0. else control
        
        ship.Control.Pitch <- sanitizeInput pitchCommand |> float32U
        ship.Control.Roll <- sanitizeInput rollCommand |> float32U
        ship.Control.Yaw <- sanitizeInput yawCommand |> float32U

    let killRotation (dt: float<s>) =
        let angvel = orbitalAngularVelocityS.Value
        let axis, speed = Vec3.norm angvel, Vec3.mag angvel

        let shipAxis = Vec3.pack<1> <| ksc.TransformDirection(Vec3.unpack axis, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
        
        // Try to stop the angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        let horizon = 5. * dt

        // compute the angular acceleration that is necessary to stop within the desired horizon
        let targetAngAccel = shipAxis * speed / horizon
        
        controlAngularAcceleration targetAngAccel
    
    let lockTarget (dt: float<s>) (shipTarget: vec3<1>) =
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

        printfn "EXIST %.3f EXCESS %.3f" (Vec3.mag existingTargetAngVel) (Vec3.mag excessRotation)

        printfn "TGT %O ACL %O" targetAxis accel

        // compute available acceleration towards target
        let towardsTargetAccel = Vec3.project accel targetAxis

        // compute commands
        let targetAccelMag = Vec3.mag towardsTargetAccel
        let targetAngVelMag = Vec3.mag existingTargetAngVel
        
        printfn "TTA %.3f ACC %.3f ACP %.3f ACN %.3f" targetAccelMag (Vec3.mag accel) (Vec3.mag accelPos) (Vec3.mag accelNeg)

        let fullStopTime = Vec3.mag existingTargetAngVel / targetAccelMag
        let onTargetTime = - targetAngVelMag / targetAccelMag + sqrt ((targetAngVelMag * targetAngVelMag) / (targetAccelMag * targetAccelMag) + 2. * targetAngle / targetAccelMag)

        printfn "STOP %.3f %.3f" onTargetTime fullStopTime

        // Try to stop the excess angular velocity within the given horizon.
        // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        let horizon = 5. * dt

        // acceleration required for killing excess rotation
        let killExcessAccel = excessRotation / horizon
        
        let maxStopTime = 5.<s>
        let killTargetAccel =
            if onTargetTime < fullStopTime * 1.05 then
                // emergency stop
                printfn "TOO FAST!!!!!!!"
                existingTargetAngVel / horizon
            elif fullStopTime > maxStopTime * 1.05 then
                let desiredVelocity = towardsTargetAccel * maxStopTime
                let accel = desiredVelocity / horizon
            
                //// existingTargetAngVel * 0.<_>
                //// time we accelerate towards target
                //let accelTime = (sqrt 2. * sqrt (2. * targetAccelMag * targetAngle + targetAngVelMag*targetAngVelMag) - 2. * targetAngVelMag) / (2. * targetAccelMag)
        
                //// time we decelerate towards target
                //let deccelTime = (targetAngVelMag + accelTime * targetAccelMag) / targetAccelMag
        
                //printfn "ACCL %.3f DCCL %.3f" accelTime deccelTime

                accel
            else
                Vec3.zero


        controlAngularAcceleration (killExcessAccel + killTargetAccel)

        //// check if we're too fast already
        //if accelTime < 0.<s> then
        //    printfn "WARN too fast, killing rotation first"
        //    // if so, slow down first
        //    killRotation dt
        //else
        //    // Try to stop the excess angular velocity within the given horizon.
        //    // The horizon is chosen so that we won't overshoot (as we can only change input in discrete intervals)
        //    let horizon = 5. * dt

        //    // acceleration required for killing excess rotation
        //    let killExcessAccel = excessRotation / horizon

        //    // only use a percentage of available acceleration for rotating towards target
        //    let safetyMargin = 0.9

        //    let desiredTowardsTargetAccel =
        //        if accelTime < horizon then
        //            printfn "DECCELERATING"
        //            -towardsTargetAccel
        //        else
        //            printfn "ACCELERATING"
        //            towardsTargetAccel * safetyMargin
    
        //    controlAngularAcceleration (killExcessAccel - desiredTowardsTargetAccel)

    member val Mode = SteeringMode.KillRotation with get, set

    member this.Update(ut: float<s>): unit =
        if ut > lastUT then 
            let dt = ut - lastUT
            lastUT <- ut
            match this.Mode with
            | KillRotation -> killRotation dt
            | LockTarget (target, ref) -> lockTarget dt (Vec3.pack <| ksc.TransformDirection(Vec3.unpack target, ref, ship.ReferenceFrame))

    interface IDisposable with
        member this.Dispose() = ()
