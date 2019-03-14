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
                
    let deltaV (exhaustVelocity: float<m/s>) (initialMass: float<kg>) (finalMass: float<kg>) =
        exhaustVelocity * log (initialMass / finalMass)

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

    type ManeuverDirections = {
        prograde: vec3<1>;
        radial: vec3<1>; // outwards
        normal: vec3<1>
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

        let pos = rt * { x = cos vt; y = 0.; z = sin vt }
        let vel = sqrt (mu * a) / rt * { x = - sinRad Et; y = 0.; z = sqrt (1. - square e) * cosRad Et }

        let i = elements.inclination
        let lan = elements.longitudeOfAscendingNode
        let arg = elements.argumentOfPeriapsis

        let transform v = Mat3x3.rotY -lan * (Mat3x3.rotX -i * (Mat3x3.rotY -arg * v))
        let r = transform pos
        let v = transform vel

        (r, v)


    // Contains nasty edge cases, not needed at the moment
    //let orbitalElements (mu: float<m^3/s^2>) (r: vec3<m>) (v: vec3<m/s>) (t: float<s>): OrbitalElements =
    //    let h = Vec3.cross r v // orbital momentum
    //    let eccentricityVector = Vec3.cross v h / mu - Vec3.norm r
    //    let n = Vec3.cross Vec3.unitY h // pointing towards ascending node
    //    let trueAnomaly =
    //        if Vec3.dot r v >= 0.<_> then
    //            Vec3.angle eccentricityVector r
    //        else
    //            2.<rad> * pi - Vec3.angle eccentricityVector r
    //    let inclination = 1.<rad> * acos (h.y / Vec3.mag h) - 90.<deg> / degPerRad
    //    let eccentricity = Vec3.mag eccentricityVector
    //    let eccentricAnomaly = 2.<rad> * atan (tan (trueAnomaly / 2.<rad>) / sqrt ((1. + eccentricity) / (1. - eccentricity)))
    //    let longitudeOfAscendingNode =
    //        let angle = acos (n.x / Vec3.mag n) * 1.<rad>
    //        if n.z >= 0.<_> then
    //            angle
    //        else
    //            2.<rad> * pi - angle
    //    let argumentOfPeriapsis =
    //        if eccentricityVector.y >= 0. then
    //            Vec3.angle n eccentricityVector
    //        else
    //            2.<rad> * pi - Vec3.angle n eccentricityVector
    //    let meanAnomaly = eccentricAnomaly - eccentricity * sinRad eccentricAnomaly * 1.<rad>
    //    let a = 1. / (2. / Vec3.mag r - Vec3.magSq v / mu)
    //    { semiMajorAxis = a
    //    ; eccentricity = eccentricity
    //    ; argumentOfPeriapsis = argumentOfPeriapsis
    //    ; longitudeOfAscendingNode = longitudeOfAscendingNode
    //    ; inclination = inclination
    //    ; meanAnomalyAtEpoch = meanAnomaly
    //    ; epoch = t
    //    }

    let orbitalPeriod (mu: float<m^3/s^2>) (sma: float<m>): float<s> = 2. * Util.pi * sqrt (Util.cube sma / mu)

    let semiMajorAxisFromPeriod (mu: float<m^3/s^2>) (period: float<s>): float<m> = cbrt (square (period / (2. * Util.pi)) * mu)

    let maneuverDirections (pos: vec3<m>) (vel: vec3<m/s>): ManeuverDirections =
        let prograde = Vec3.norm vel
        let radialTmp = Vec3.norm pos
        let normal = Vec3.norm (Vec3.cross prograde radialTmp)
        let radial = -Vec3.cross prograde normal
        { prograde = prograde
        ; radial = radial
        ; normal = normal
        }
        