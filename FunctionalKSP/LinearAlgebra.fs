namespace FunctionalKSP.LinearAlgebra

[<Struct>]
type vec3<[<Measure>] 'u> = 
    { x: float<'u>
      y: float<'u>
      z: float<'u> }

    static member (+)(v1: vec3<'u>, v2: vec3<'u>) =
        { x = v1.x + v2.x; y = v1.y + v2.y; z = v1.z + v2.z }
    static member (-)(v1: vec3<'u>, v2: vec3<'u>) =
        { x = v1.x - v2.x; y = v1.y - v2.y; z = v1.z - v2.z }
    static member (~-)(v2: vec3<'u>) =
        { x = - v2.x; y = - v2.y; z = - v2.z }
    static member (*)(v1: vec3<'u>, s: float<'v>) =
        { x = v1.x * s; y = v1.y * s; z = v1.z * s}
    static member (/)(v1: vec3<'u>, s: float<'v>) =
        { x = v1.x / s; y = v1.y / s; z = v1.z / s}
    static member (*)(s: float<'v>, v1: vec3<'u>) =
        { x = v1.x * s; y = v1.y * s; z = v1.z * s}
    
    override this.ToString() = sprintf "V(%.3f, %.3f, %.3f)" this.x this.y this.z

module Vec3 =
    open FunctionalKSP.Units

    let pack<[<Measure>] 'u> (x: float, y: float, z: float): vec3<'u> =
        let asU v : float<'u> = LanguagePrimitives.FloatWithMeasure v
        { x = asU x; y = asU y; z = asU z }
        
    let unpack<[<Measure>] 'u> (v: vec3<'u>): float * float * float =
        float v.x, float v.y, float v.z

    let dot<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float<'u * 'v> =
        v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    let magSq<[<Measure>] 'u> (v: vec3<'u>): float<'u^2> = dot v v

    let mag<[<Measure>] 'u> (v: vec3<'u>): float<'u> = sqrt (magSq v)

    let norm<[<Measure>] 'u> (v: vec3<'u>): vec3<1> = v / mag v

    let cosAngle<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float = dot (norm v1) (norm v2)
    
    let angle<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float<rad> = acos (cosAngle v1 v2) * 1.<rad>

    let zip<[<Measure>] 'u, [<Measure>] 'v, [<Measure>] 'w> (f: float<'u> -> float<'v> -> float<'w>) (v1: vec3<'u>) (v2: vec3<'v>): vec3<'w> =
        { x = f v1.x v2.x; y = f v1.y v2.y; z = f v1.z v2.z }

    let project<[<Measure>] 'u, [<Measure>] 'v> (v: vec3<'u>) (axis: vec3<1>): vec3<'u> =
        axis * dot v axis
    
    let exclude<[<Measure>] 'u> (v: vec3<'u>) (axis: vec3<1>) =
        v - project v axis

    let cross<[<Measure>] 'u, [<Measure>] 'v> (a: vec3<'u>) (b: vec3<'v>): vec3<'u * 'v> =
        { x = a.y * b.z - a.z * b.y; y = a.z * b.x - a.x * b.z; z = a.x * b.y - a.y * b.x }
   
    let compassHeading (north: vec3<1>) (east: vec3<1>) (v: vec3<1>) =
        let vnorth = dot v north
        let veast = dot v east
        let angle = 90.<deg> - atan2 vnorth veast * 1.<rad> * degPerRad
        if angle < 0.<deg>
            then angle + 360.<deg>
            else angle
           
    let unitX: vec3<1> = pack (1., 0., 0.)
    let unitY: vec3<1> = pack (0., 1., 0.)
    let unitZ: vec3<1> = pack (0., 0., 1.)

    let zero<[<Measure>] 'u> = pack<'u> (0., 0., 1.)

[<Struct>]
type euler<[<Measure>] 'u> = 
    { pitch: float<'u>
      roll: float<'u>
      yaw: float<'u> }

    static member (+)(v1: euler<'u>, v2: euler<'u>) =
        { pitch = v1.pitch + v2.pitch; roll = v1.roll + v2.roll; yaw = v1.yaw + v2.yaw }
    static member (-)(v1: euler<'u>, v2: euler<'u>) =
        { pitch = v1.pitch - v2.pitch; roll = v1.roll - v2.roll; yaw = v1.yaw - v2.yaw }
    static member (~-)(v2: euler<'u>) =
        { pitch = - v2.pitch; roll = - v2.roll; yaw = - v2.yaw }
    static member (*)(v1: euler<'u>, s: float<'v>) =
        { pitch = v1.pitch * s; roll = v1.roll * s; yaw = v1.yaw * s}
    static member (/)(v1: euler<'u>, s: float<'v>) =
        { pitch = v1.pitch / s; roll = v1.roll / s; yaw = v1.yaw / s}
    static member (*)(s: float<'v>, v1: euler<'u>) =
        { pitch = v1.pitch * s; roll = v1.roll * s; yaw = v1.yaw * s}

    override this.ToString() = sprintf "R(%.3f, %.3f, %.3f)" this.pitch this.roll this.yaw

module Rotation =
    open FunctionalKSP.Units
    open System
    
    let packEuler<[<Measure>] 'u> (x: float, y: float, z: float): euler<'u> =
        let asU v : float<'u> = LanguagePrimitives.FloatWithMeasure v
        { pitch = asU x; roll = asU y; yaw = asU z }

    /// Compute the axis and angle between two orientation vectors.
    /// The must be normalized and not collinear.
    let axisAngle (v1: vec3<1>) (v2: vec3<1>): vec3<1> * float<rad> =
        let axis = Vec3.cross v1 v2
        let angle = acos (Vec3.dot v1 v2)
        Vec3.norm axis, angle * 1.<rad>

    let axisAngleToEuler (axis: vec3<1>) (angle: float<rad>): euler<rad> =
        let { x = x; y = y; z = z } = axis
        let s = sin (angle / 1.<rad>)
        let c = cos (angle / 1.<rad>)
        let t = 1. - c
        
        if ((x*y*t + z*s) > 0.998) then // north pole singularity detected
            printfn "North pole"
            { roll = 2.<rad> * atan2 (x * sin (angle/2.<rad>)) (cos (angle/2.<rad>));
              yaw = Math.PI * 0.5<rad>;
              pitch = 0.<rad> }
        elif ((x*y*t + z*s) < -0.998) then // south pole singularity detected
            printfn "South pole"
            { roll = -2.<rad> * atan2 (x * sin (angle/2.<rad>)) (cos (angle/2.<rad>));
              yaw = -Math.PI * 0.5<rad>;
              pitch = 0.<rad> }
        else
            printfn "no pole"
            { roll = atan2 (y * s- x * z * t) (1. - (y*y+ z*z ) * t) * 1.<rad>;
              yaw = asin (x * y * t + z * s) * 1.<rad>;
              pitch = atan2 (x * s - y * z * t) (1. - (x*x + z*z) * t) * 1.<rad> }

              (*
Heading = rotation about y axis = roll
Attitude = rotation about z axis = yaw
Bank = rotation about x axis = pitch
              *)
              // solve r_1 = r_0 + w_0 * (t - (at + w)/d) + 0.5 a t^2 + (a * t) * (- (at + w)/d) + 0.5 * d + (- (at + w)/d)^2 for t