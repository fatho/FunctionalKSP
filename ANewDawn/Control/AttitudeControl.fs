namespace ANewDawn.Control

open ANewDawn.Units
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Extensions
open ANewDawn
open KRPC.Client.Services.SpaceCenter

/// Control inputs that are applied to the controlled vessel.
type AttitudeControls = {
    pitch: float;
    roll: float;
    yaw: float;
}

/// Steering input in some reference frame
type ShipAttitude = {
    /// The direction the front vessel should be pointing at
    forward: vec3<1>;
    /// The direction the top of the vessel should be pointing at
    top: option<vec3<1>>;
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
type AttitudeController(torque: IStream<vec3<N m>>, momentOfInertia: IStream<vec3<kg m^2>>, angularVelocity: IStream<vec3<rad/s>>) =
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
    
    member this.UpdateForAngularVelocity(sampleTime: float<s>, pitchVel: float<rad/s>, rollVel: float<rad/s>, yawVel: float<rad/s>): vec3<1> =
        let controlTorque = torque.Value
        let moi = momentOfInertia.Value
        let angularVel = angularVelocity.Value

        let tgtPitchTorque = this.PitchTorqueLoop.Update(sampleTime, angularVel.x,
                                pitchVel, moi.x, controlTorque.x)
        let tgtYawTorque = this.YawTorqueLoop.Update(sampleTime, angularVel.z,
                                yawVel, moi.z, controlTorque.z)
        let tgtRollTorque = this.RollTorqueLoop.Update(sampleTime, angularVel.y,
                                rollVel, moi.y, controlTorque.y)
                                
        let snapToEpsilon v = if abs v < EPSILON then 0. else v

        let clampPitch = 2. * max (abs this.LastPitchInput) 0.05
        this.LastPitchInput <- snapToEpsilon <| clamp -clampPitch clampPitch (tgtPitchTorque / controlTorque.x)

        let clampYaw = 2. * max (abs this.LastYawInput) 0.05
        this.LastYawInput <- snapToEpsilon <| clamp -clampYaw clampYaw (tgtYawTorque / controlTorque.z)
        
        let clampRoll = 2. * max (abs this.LastRollInput) 0.05
        this.LastRollInput <- snapToEpsilon <| clamp -clampRoll clampRoll (tgtRollTorque / controlTorque.y)

        { x = this.LastPitchInput; y = this.LastRollInput; z = this.LastYawInput }

    /// Update the steering controls based on the new time and target. The target must be given
    /// in the vessel's reference frame.
    member this.UpdateForAttitude(sampleTime: float<s>, target: ShipAttitude): vec3<1> =
        let vesselForward = Vec3.unitY
        let vesselStarboard = Vec3.unitX
        let vesselTop = - Vec3.unitZ

        let phi =
            Vec3.angle vesselForward target.forward * (
                if Vec3.angle vesselTop target.forward > 90.<deg> / degPerRad then -1. else 1.)

        let targetForwardNoStarboard = Vec3.exclude target.forward vesselStarboard
        let phiPitch =
            Vec3.angle vesselForward targetForwardNoStarboard * (
                if Vec3.angle vesselTop targetForwardNoStarboard > 90.<deg> / degPerRad then -1. else 1.)

        let targetForwardNoTop = Vec3.exclude target.forward vesselTop
        let phiYaw =
            Vec3.angle vesselForward targetForwardNoTop * (
                if Vec3.angle vesselStarboard targetForwardNoTop > 90.<deg> / degPerRad then -1. else 1.)
                
        let phiRoll =
            match target.top with
            | None -> 0.<rad>
            | Some(top) ->
                let targetTopNoForward = Vec3.exclude top vesselForward
                Vec3.angle vesselTop targetTopNoForward * (
                    if Vec3.angle vesselStarboard targetTopNoForward > 90.<deg> / degPerRad then -1. else 1.)
        
        let controlTorque = torque.Value
        let moi = momentOfInertia.Value
        let angularVel = angularVelocity.Value

        let maxOmega = this.MaxStoppingTime * Inertia.angularAcceleration moi controlTorque

        this.PitchRateLoop.ExtraUnwind <- true
        this.YawRateLoop.ExtraUnwind <- true
        this.RollRateLoop.ExtraUnwind <- true

        let tgtPitchOmega = this.PitchRateLoop.Update(sampleTime, -phiPitch, 0.<_>, maxOmega.x)
        let tgtYawOmega = this.YawRateLoop.Update(sampleTime, -phiYaw, 0.<_>, maxOmega.z)
        let tgtRollOmega =
            if target.top.IsNone || abs phi > this.RollControlRange / degPerRad then
                this.RollRateLoop.ResetI()
                0.<rad/s>
            else
                this.RollRateLoop.Update(sampleTime, -phiRoll, 0.<_>, maxOmega.y)

        let tgtPitchTorque = this.PitchTorqueLoop.Update(sampleTime, angularVel.x,
                                tgtPitchOmega, moi.x, controlTorque.x)
        let tgtYawTorque = this.YawTorqueLoop.Update(sampleTime, angularVel.z,
                                tgtYawOmega, moi.z, controlTorque.z)
        let tgtRollTorque = this.RollTorqueLoop.Update(sampleTime, angularVel.y,
                                tgtRollOmega, moi.y, controlTorque.y)
                                
        let snapToEpsilon v = if abs v < EPSILON then 0. else v

        let clampPitch = 2. * max (abs this.LastPitchInput) 0.05
        this.LastPitchInput <- snapToEpsilon <| clamp -clampPitch clampPitch (tgtPitchTorque / controlTorque.x)

        let clampYaw = 2. * max (abs this.LastYawInput) 0.05
        this.LastYawInput <- snapToEpsilon <| clamp -clampYaw clampYaw (tgtYawTorque / controlTorque.z)
        
        let clampRoll = 2. * max (abs this.LastRollInput) 0.05
        this.LastRollInput <- snapToEpsilon <| clamp -clampRoll clampRoll (tgtRollTorque / controlTorque.y)

        { x = this.LastPitchInput; y = this.LastRollInput; z = this.LastYawInput }


module AttitudeControl = 
    open ANewDawn.Mission
    open ANewDawn.StreamExtensions

    type AttitudeControlProfile = Heavy  | Medium | Light
            

    /// Sends a sequence of steering commands based on the sequence of attitudes.
    /// Every clock tick, the sequence is advanced, until it ends.
    /// At the end of the sequence, the steering commands are reset to zero.
    let loopWithProfile (mission: Mission) (profile: AttitudeControlProfile) (referenceFrame: ReferenceFrame) (attitudes: seq<ShipAttitude>) =
        let ship = mission.ActiveVessel
        use availableTorqueS = mission.Streams.UseStream<N m>(fun () -> ship.AvailableTorque)
        use moiS = mission.Streams.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
        use orbitalAngularVelocityS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))

        let combineTorques (pos: vec3<N m>, neg: vec3<N m>): vec3<N m> = Vec3.zip (fun a b -> min (abs a) (abs b)) pos neg
        let transformAngVel av = -(Vec3.pack<rad/s> <| mission.SpaceCenter.TransformDirection(Vec3.unpack av, ship.OrbitalReferenceFrame, ship.ReferenceFrame))

        let controller = new AttitudeController(availableTorqueS.Map(combineTorques), moiS, orbitalAngularVelocityS.Map(transformAngVel))

        // TODO: base control profile on available torque
        printfn "Controlling attitude with profile %O" profile
        match profile with
        | Light -> ()
        | Medium -> ()
        | Heavy -> do
            // Make configuration adjustments for heavier vessel
            controller.MaxStoppingTime <- 20.<s>
            //controller.PitchRateLoop.Kd <- 0.1
            //controller.YawRateLoop.Kd <- 0.1
            
        for attitude in attitudes do
            let shipTarget = Vec3.pack <| mission.SpaceCenter.TransformDirection(Vec3.unpack attitude.forward, referenceFrame, ship.ReferenceFrame)
            let shipUp = Option.map (fun top -> Vec3.pack <| mission.SpaceCenter.TransformDirection(Vec3.unpack top, referenceFrame, ship.ReferenceFrame)) attitude.top
            let controls = controller.UpdateForAttitude(mission.UniversalTime, { forward = shipTarget; top = shipUp })
            ship.Control.Pitch <- float32U controls.x
            ship.Control.Yaw <- float32U controls.z
            ship.Control.Roll <- float32U controls.y
            mission.Tick() |> ignore
        
        ship.Control.Pitch <- 0.f
        ship.Control.Yaw <- 0.f
        ship.Control.Roll <- 0.f
        
    /// Sends a sequence of steering commands based on the sequence of attitudes.
    /// Every clock tick, the sequence is advanced, until it ends.
    /// At the end of the sequence, the steering commands are reset to zero.
    let loop (mission: Mission) (referenceFrame: ReferenceFrame) (attitudes: seq<ShipAttitude>) =
        loopWithProfile mission AttitudeControlProfile.Medium referenceFrame attitudes

    /// Sends a sequence of steering commands based on the sequence of attitudes.
    /// Every clock tick, the sequence is advanced, until it ends.
    /// At the end of the sequence, the steering commands are reset to zero.
    let loopAngVel (mission: Mission) (angVels: seq<vec3<rad/s>>) =
        let ship = mission.ActiveVessel
        use availableTorqueS = mission.Streams.UseStream<N m>(fun () -> ship.AvailableTorque)
        use moiS = mission.Streams.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
        use orbitalAngularVelocityS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))

        let combineTorques (pos: vec3<N m>, neg: vec3<N m>): vec3<N m> = Vec3.zip (fun a b -> min (abs a) (abs b)) pos neg
        let transformAngVel av = -(Vec3.pack<rad/s> <| mission.SpaceCenter.TransformDirection(Vec3.unpack av, ship.OrbitalReferenceFrame, ship.ReferenceFrame))

        let controller = new AttitudeController(availableTorqueS.Map(combineTorques), moiS, orbitalAngularVelocityS.Map(transformAngVel))

        for angVel in angVels do
            let controls = controller.UpdateForAngularVelocity(mission.UniversalTime, angVel.x, angVel.y, angVel.z)
            ship.Control.Pitch <- float32U controls.x
            ship.Control.Yaw <- float32U controls.z
            ship.Control.Roll <- float32U controls.y
            mission.Tick() |> ignore
        
        ship.Control.Pitch <- 0.f
        ship.Control.Yaw <- 0.f
        ship.Control.Roll <- 0.f