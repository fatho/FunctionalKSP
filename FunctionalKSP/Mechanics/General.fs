namespace FunctionalKSP.Mechanics

open FunctionalKSP.Math
open FunctionalKSP.Units

module Inertia =
    let angularAcceleration (moi: vec3<kg*m^2>) (torque: vec3<N * m>): vec3<rad/s^2> =
        1.0<rad> * Vec3.zip (/) torque moi

    let torque (moi: vec3<kg*m^2>) (angularAccel: vec3<rad/s^2>): vec3<N * m> =
        1.0<1/rad> * Vec3.zip (*) angularAccel moi