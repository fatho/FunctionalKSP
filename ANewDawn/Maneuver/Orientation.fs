namespace ANewDawn.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Control
open ANewDawn.Mission

module Orientation =
    /// Perform a maneuver that orients the vessel in the given direction, does not perform roll
    let orient (mission: Mission) (ref: ReferenceFrame) (direction: vec3<1>) (thresholdPos: float<deg>) (thresholdVel: float<deg/s>) (timeout: float<s>): float<deg> * float<deg/s> =
        let ship = mission.ActiveVessel
        use angVelS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.Orbit.Body.NonRotatingReferenceFrame))
        use shipFacingS = mission.Streams.UseStream<1>(fun () -> ship.Direction(ref))

        //use steering = new KosSteering()
        
        let tMax = mission.UniversalTime + timeout
        
        let mutable remainingAngle = infinity.As<deg>()
        let mutable remainingAngularVel = infinity.As<deg/s>()

        AttitudeControl.loop mission ref <| seq {
            while not (mission.UniversalTime >= tMax || (remainingAngle <= thresholdPos && remainingAngularVel <= thresholdVel)) do
                yield { forward = direction; top = None }
                remainingAngle <- Vec3.angle shipFacingS.Value direction * degPerRad
                remainingAngularVel <- Vec3.mag angVelS.Value * degPerRad
        }
        remainingAngle, remainingAngularVel

    /// Perform a maneuver that kills all rotation in the orbital reference frame.
    /// Returns the residual angular velocity that was not killed.
    let killRotation (mission: Mission) (threshold: float<deg/s>) (timeout: float<s>): vec3<deg/s> =
        let ship = mission.ActiveVessel
        use angVelS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.Orbit.Body.NonRotatingReferenceFrame))
        let tMax = mission.UniversalTime + timeout
        let mutable remainingAngularVel = infinity.As<deg/s>()
        
        AttitudeControl.loopAngVel mission <| seq {
            while not (mission.UniversalTime >= tMax || remainingAngularVel <= threshold) do
                yield Vec3.zero<_>
                remainingAngularVel <- Vec3.mag angVelS.Value * degPerRad
        }
        angVelS.Value * degPerRad
