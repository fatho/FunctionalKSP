namespace ANewDawn.Math

[<Struct>]
type mat3x3<[<Measure>] 'u> =
    { a11: float<'u>; a12: float<'u>; a13: float<'u>;
      a21: float<'u>; a22: float<'u>; a23: float<'u>;
      a31: float<'u>; a32: float<'u>; a33: float<'u>; }

    static member (*)(m: mat3x3<'u>, v: vec3<'v>) =
        { x = m.a11 * v.x + m.a12 * v.y + m.a13 * v.z
          y = m.a21 * v.x + m.a22 * v.y + m.a23 * v.z
          z = m.a31 * v.x + m.a32 * v.y + m.a33 * v.z }
    
    /// override this.ToString() = sprintf "V(%.3f, %.3f, %.3f)" this.x this.y this.z

module Mat3x3 =
    open ANewDawn.Units
    open ANewDawn.Math.Util

    let rotX (angle: float<rad>) =
        { a11 = 1.; a12 = 0.;           a13 = 0.;
          a21 = 0.; a22 = cosRad angle; a23 = - sinRad angle;
          a31 = 0.; a32 = sinRad angle; a33 = cosRad angle; }
          
    let rotY (angle: float<rad>) =
        { a11 =   cosRad angle; a12 = 0.; a13 = sinRad angle;
          a21 =   0.;           a22 = 1.; a23 = 0.;
          a31 = - sinRad angle; a32 = 0.; a33 = cosRad angle; }
          
    let rotZ (angle: float<rad>) =
        { a11 = cosRad angle; a12 = - sinRad angle; a13 = 0.;
          a21 = sinRad angle; a22 =   cosRad angle; a23 = 0.;
          a31 = 0.;           a32 =   0.;           a33 = 1.; }

    let unit: mat3x3<1> = 
        { a11 = 1.; a12 = 0.; a13 = 0.;
          a21 = 0.; a22 = 1.; a23 = 0.;
          a31 = 0.; a32 = 0.; a33 = 1.; }
