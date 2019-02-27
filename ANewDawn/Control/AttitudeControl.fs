namespace ANewDawn.Control

open ANewDawn.Units
open ANewDawn.Math
open ANewDawn.Mechanics
open ANewDawn.Extensions
open ANewDawn

/// Control inputs that are applied to the controlled vessel.
type AttitudeControls = {
    pitch: float;
    roll: float;
    yaw: float;
}

/// Steering input in ship reference frame
type SteeringTarget = {
    forward: vec3<1>;
    top: vec3<1>;
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

    member this.UpdatePrediction(sampleTime: float<s>, target: SteeringTarget): vec3<1> =
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
                
        let targetTopNoForward = Vec3.exclude target.top vesselForward
        let phiRoll =
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
            if abs phi > this.RollControlRange / degPerRad then
                this.RollRateLoop.ResetI()
                0.<rad/s>
            else
                this.RollRateLoop.Update(sampleTime, -phiRoll, 0.<_>, maxOmega.y)

        // NOTE: coordinate system is apaprently different from kOS, so this code requires some
        // random negations.
        // TODO: find out why
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