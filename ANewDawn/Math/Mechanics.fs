namespace ANewDawn.Math

open ANewDawn.Units
open ANewDawn.Math.Util

/// Vis-viva equation (https://en.wikipedia.org/wiki/Vis-viva_equation)
/// v^2 = GM (2/r - 1/a) where r is the current distance from the bodies center and a the semi-major=axis of the orbit.
module VisViva =
    let orbitalVelocitySq (mu: float<m^3/s^2>) (r: float<m>) (sma: float<m>) =
        mu * (2. / r - 1. / sma)

    /// Compute the orbital velocity on an elliptical orbit with semi-major axis `sma` around
    /// a body with a gravitational parameter `mu` at the current distance `r` from the bodies center.
    /// Computes v^2 = GM (2/r - 1/a) where r is the current distance from the bodies center and a the semi-major=axis of the orbit.
    let orbitalVelocity  (mu: float<m^3/s^2>) (r: float<m>) (sma: float<m>) =
        sqrt (orbitalVelocitySq mu r sma)

/// Tsiolkovsky rocket equation
/// dv = v_e ln(m_0/m_f)
module Tsiolkovsky =
    let finalMass (dv: float<m/s>) (exhaustVelocity: float<m/s>) (initialMass: float<kg>) =
        initialMass / exp (dv / exhaustVelocity)

module Inertia =
    let angularAcceleration (moi: vec3<kg*m^2>) (torque: vec3<N * m>): vec3<rad/s^2> =
        1.0<rad> * Vec3.zip (/) torque moi

    let torque (moi: vec3<kg*m^2>) (angularAccel: vec3<rad/s^2>): vec3<N * m> =
        1.0<1/rad> * Vec3.zip (*) angularAccel moi

module Kepler =
    open System

    type OrbitalElements = {
        semiMajorAxis: float<m>;
        eccentricity: float<1>;
        argumentOfPeriapsis: float<rad>;
        longitudeOfAscendingNode: float<rad>;
        inclination: float<rad>;
        meanAnomalyAtEpoch: float<rad>;
        epoch: float<s>;
    }

    let stateVectors (mu: float<m^3/s^2>) (elements: OrbitalElements) (t: float<s>): vec3<m> * vec3<m/s> =
        let dt = t - elements.epoch
        let e = elements.eccentricity
        let a = elements.semiMajorAxis
        // determine mean anomaly at t
        let Mt = (elements.meanAnomalyAtEpoch + 1.<rad> * dt * sqrt (mu / cube a)) % (2.<rad> * pi)

        // determine eccentric anomaly at t
        let tolerance = 1e-14<rad>

        let mutable Eold = Mt
        let mutable err = Double.PositiveInfinity.As<rad>()

        while err > tolerance do
            let Enew = Eold - (Eold - 1.<rad> * e * sinRad Eold - Mt) / (1. - e * cosRad Eold)
            err <- abs (Enew - Eold)
            Eold <- Enew

        let Et = Eold
        let vt = 2. * atan2 (sqrt (1. + e) * sinRad (Et / 2.)) (sqrt (1. - e) * cosRad (Et / 2.))
        let rt = a * (1. - e * cosRad Et)

        printfn "M = %.3f E = %.3f v = %.3f r = %.3f" Mt Et vt rt

        let pos = rt * { x = cos vt; y = 0.; z = sin vt }
        let vel = sqrt (mu * a) / rt * { x = - sinRad Et; y = 0.; z = sqrt (1. - square e) * cosRad Et }

        let i = elements.inclination
        let lan = elements.longitudeOfAscendingNode
        let arg = elements.argumentOfPeriapsis

        let transform v = Mat3x3.rotY -lan * (Mat3x3.rotX -i * (Mat3x3.rotY -arg * v))
        let r = transform pos
        let v = transform vel

        (r, v)

