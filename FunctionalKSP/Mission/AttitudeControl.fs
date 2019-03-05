namespace FunctionalKSP.Mission

open FunctionalKSP.Units
open FunctionalKSP.Math

/// Control inputs that are applied to the controlled vessel.
type AttitudeControls = {
    pitch: float;
    roll: float;
    yaw: float;
}


/// Steering input in ship reference frame
type AtmosphericSteeringInput = {
    sampleTime: float<s>;
    targetForward: vec3<1>;
    targetTop: vec3<1>;
    controlTorque: vec3<N m>;
    momentOfInertia: vec3<kg m^2>;
    angularVelocity: vec3<rad/s>
}


type IAttitudeController =
    abstract member Update: float<s> -> AttitudeControls
