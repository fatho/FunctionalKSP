namespace ANewDawn.Math

open ANewDawn.Units

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