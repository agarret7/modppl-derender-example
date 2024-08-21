use crate::types::*;


/* init */ 

pub const fn mat4_zero() -> Mat4 { [[0.0; 4]; 4] }
pub const fn mat4_id() -> Mat4 { [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]] }
pub const fn vec3_zero() -> Vec3 { [0.0; 3] }
pub const fn vec4_zero() -> Vec4 { [0.0; 4] }
pub const fn quat_id() -> Quat { [0.0,0.0,0.0,1.0] }
pub const fn pose_id() -> Pose {
    let p = vec3_zero();
    let q = quat_id();
    [p[0],p[1],p[2],q[0],q[1],q[2],q[3]]
}


/* much of the following was taken from cglm (https://github.com/recp/cglm)

   The MIT License (MIT)
  
   Copyright (c) 2015 Recep Aslantas <info@recp.me>
  
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
  
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
  
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#[inline]
pub fn unproject_inv(pos: Vec3, inv_m: Mat4, vp: Vec4) -> Vec3 {
    let mut v = vec4_zero();

    v[0] = 2.0 * (pos[0] - vp[0]) / vp[2] - 1.0;
    v[1] = 2.0 * (pos[1] - vp[1]) / vp[3] - 1.0;
    v[2] = pos[2];
    v[3] = 1.0;

    let v = mat4_mulv(inv_m, v);
    let v = vec4_scale(&v, 1.0 / v[3]);
    let dest = [v[0], v[1], v[2]];
    dest
}

#[inline]
pub fn ray_at(ray_origin: Vec3, ray_dir: Vec3, distance: f32) -> Vec3 {
    vec3_add(ray_origin, vec3_scale(&ray_dir, distance))
}

#[inline]
pub fn perspective(fovy: f32, aspect_ratio: f32, nearz: f32, farz: f32) -> Mat4 {
    let mut out = mat4_zero();

    let f = 1.0 / (fovy * 0.5).tan();
    let fnorm = 1.0 / (nearz - farz);

    out[0][0] = f / aspect_ratio;
    out[1][1] = f;
    out[2][2] = (nearz + farz) * fnorm;
    out[2][3] = -1.0;
    out[3][2] = 2.0 * nearz * farz * fnorm;

    out
}

#[inline]
pub fn euler_xyz(angles: Vec3) -> Quat {
    let sx   = angles[0].sin(); let cx = angles[0].cos();
    let sy   = angles[1].sin(); let cy = angles[1].cos();
    let sz   = angles[2].sin(); let cz = angles[2].cos();

    let czsx = cz * sx;
    let cxcz = cx * cz;
    let sysz = sy * sz;

    let mut m = mat4_zero();

    m[0][0] =  cy * cz;
    m[0][1] =  czsx * sy + cx * sz;
    m[0][2] = -cxcz * sy + sx * sz;
    m[1][0] = -cy * sz;
    m[1][1] =  cxcz - sx * sysz;
    m[1][2] =  czsx + cx * sysz;
    m[2][0] =  sy;
    m[2][1] = -cy * sx;
    m[2][2] =  cx * cy;
    m[0][3] =  0.0;
    m[1][3] =  0.0;
    m[2][3] =  0.0;
    m[3][0] =  0.0;
    m[3][1] =  0.0;
    m[3][2] =  0.0;
    m[3][3] =  1.0;

    let trace = m[0][0] + m[1][1] + m[2][2];
    let r: f32;
    let rinv: f32;
    let mut out = quat_id();
    if trace >= 0.0 {
        r       = (1.0 + trace).sqrt();
        rinv    = 0.5 / r;

        out[0] = rinv * (m[1][2] - m[2][1]);
        out[1] = rinv * (m[2][0] - m[0][2]);
        out[2] = rinv * (m[0][1] - m[1][0]);
        out[3] = r    * 0.5;
    } else if m[0][0] >= m[1][1] && m[0][0] >= m[2][2] {
        r       = (1.0 - m[1][1] - m[2][2] + m[0][0]).sqrt();
        rinv    = 0.5 / r;

        out[0] = r    * 0.5;
        out[1] = rinv * (m[0][1] + m[1][0]);
        out[2] = rinv * (m[0][2] + m[2][0]);
        out[3] = rinv * (m[1][2] - m[2][1]);
    } else if m[1][1] >= m[2][2] {
        r       = (1.0 - m[0][0] - m[2][2] + m[1][1]).sqrt();
        rinv    = 0.5 / r;

        out[0] = rinv * (m[0][1] + m[1][0]);
        out[1] = r    * 0.5;
        out[2] = rinv * (m[1][2] + m[2][1]);
        out[3] = rinv * (m[2][0] - m[0][2]);
    } else {
        r       = (1.0 - m[0][0] - m[1][1] + m[2][2]).sqrt();
        rinv    = 0.5 / r;

        out[0] = rinv * (m[0][2] + m[2][0]);
        out[1] = rinv * (m[1][2] + m[2][1]);
        out[2] = r    * 0.5;
        out[3] = rinv * (m[0][1] - m[1][0]);
    }

    out
}

#[inline]
pub fn vec3_euler_to_pose(xyz: Vec3, ypr: Vec3) -> Pose {
    let q = euler_xyz(ypr);
    [xyz[0], xyz[1], xyz[2], q[0], q[1], q[2], q[3]]
}

#[inline]
pub fn pose_to_vec3(p: Pose) -> Vec3 {
    [p[0], p[1], p[2]]
}

#[inline]
pub fn pose_to_quat(p: Pose) -> Quat {
    [p[3], p[4], p[5], p[6]]
}

#[inline]
pub fn quat_to_mat4(q: Quat, dest: &mut Mat4) {
    let norm = quat_norm(&q);
    let s    = if norm > 0.0 { 2.0 / norm } else { 0.0 };

    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];

    let xx = s * x * x;   let xy = s * x * y;   let wx = s * w * x;
    let yy = s * y * y;   let yz = s * y * z;   let wy = s * w * y;
    let zz = s * z * z;   let xz = s * x * z;   let wz = s * w * z;

    dest[0][0] = 1.0 - yy - zz;
    dest[1][1] = 1.0 - xx - zz;
    dest[2][2] = 1.0 - xx - yy;

    dest[0][1] = xy + wz;
    dest[1][2] = yz + wx;
    dest[2][0] = xz + wy;

    dest[1][0] = xy - wz;
    dest[2][1] = yz - wx;
    dest[0][2] = xz - wy;

    dest[0][3] = 0.0;
    dest[1][3] = 0.0;
    dest[2][3] = 0.0;
    dest[3][0] = 0.0;
    dest[3][1] = 0.0;
    dest[3][2] = 0.0;
    dest[3][3] = 1.0;
}

#[inline]
pub fn pose_to_mat4(p: Pose) -> Mat4 {
    let mut out = mat4_zero();
    let q = pose_to_quat(p);
    quat_to_mat4(q, &mut out);
    out[3][0] = p[0];
    out[3][1] = p[1];
    out[3][2] = p[2];
    out
}

#[inline]
pub fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub fn vec3_dot(a: &Vec3, b: &Vec3) -> f32 {
    a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}

#[inline]
pub fn vec3_scale(v: &Vec3, a: f32) -> Vec3 {
    [a*v[0], a*v[1], a*v[2]]
}

#[inline]
pub fn vec3_norm_squared(v: &Vec3) -> f32 {
    vec3_dot(&v, &v)
}

#[inline]
pub fn vec3_norm(v: &Vec3) -> f32 {
    vec3_norm_squared(v).sqrt()
}

#[inline]
pub fn vec3_normalize(v: &mut Vec3) {
    let norm = vec3_norm(v);
    v[0] /= norm; v[1] /= norm; v[2] /= norm;
}

#[inline]
pub fn vec3_mulsubs(v: Vec3, a: f32, dest: &mut Vec3) {
    dest[0] -= v[0] * a;
    dest[1] -= v[1] * a;
    dest[2] -= v[2] * a;
}

#[inline]
pub fn vec4_scale(v: &Vec4, a: f32) -> Vec4 {
    [a*v[0], a*v[1], a*v[2], a*v[3]]
}

#[inline]
pub fn vec4_norm(v: &Vec4) -> f32 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]).sqrt()
}

#[inline]
pub fn quat_norm(q: &Quat) -> f32 {
    vec4_norm(q)
}

#[inline]
pub fn mat4_scale_p(mat: &mut Mat4, v: f32) {
    mat[0][0] *= v; mat[0][1] *= v; mat[0][2] *= v; mat[0][3] *= v;
    mat[1][0] *= v; mat[1][1] *= v; mat[1][2] *= v; mat[1][3] *= v;
    mat[2][0] *= v; mat[2][1] *= v; mat[2][2] *= v; mat[2][3] *= v;
    mat[3][0] *= v; mat[3][1] *= v; mat[3][2] *= v; mat[3][3] *= v;
}

#[inline]
pub fn mat4_mulv(m: Mat4, v: Vec4) -> Vec4 {
    let mut out = vec4_zero();
    out[0] = m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2] + m[3][0]*v[3];
    out[1] = m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2] + m[3][1]*v[3];
    out[2] = m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2] + m[3][2]*v[3];
    out[3] = m[0][3]*v[0] + m[1][3]*v[1] + m[2][3]*v[2] + m[3][3]*v[3];
    out
}

#[inline]
pub fn mat4_inv(mat: Mat4) -> Mat4 {
    let mut t = [0.0; 6];
    let det: f32;

    let a = mat[0][0]; let b = mat[0][1]; let c = mat[0][2]; let d = mat[0][3];
    let e = mat[1][0]; let f = mat[1][1]; let g = mat[1][2]; let h = mat[1][3];
    let i = mat[2][0]; let j = mat[2][1]; let k = mat[2][2]; let l = mat[2][3];
    let m = mat[3][0]; let n = mat[3][1]; let o = mat[3][2]; let p = mat[3][3];

    t[0] = k * p - o * l; t[1] = j * p - n * l; t[2] = j * o - n * k;
    t[3] = i * p - m * l; t[4] = i * o - m * k; t[5] = i * n - m * j;

    let mut out = mat4_zero();

    out[0][0] =  f * t[0] - g * t[1] + h * t[2];
    out[1][0] =-(e * t[0] - g * t[3] + h * t[4]);
    out[2][0] =  e * t[1] - f * t[3] + h * t[5];
    out[3][0] =-(e * t[2] - f * t[4] + g * t[5]);

    out[0][1] =-(b * t[0] - c * t[1] + d * t[2]);
    out[1][1] =  a * t[0] - c * t[3] + d * t[4];
    out[2][1] =-(a * t[1] - b * t[3] + d * t[5]);
    out[3][1] =  a * t[2] - b * t[4] + c * t[5];

    t[0] = g * p - o * h; t[1] = f * p - n * h; t[2] = f * o - n * g;
    t[3] = e * p - m * h; t[4] = e * o - m * g; t[5] = e * n - m * f;

    out[0][2] =  b * t[0] - c * t[1] + d * t[2];
    out[1][2] =-(a * t[0] - c * t[3] + d * t[4]);
    out[2][2] =  a * t[1] - b * t[3] + d * t[5];
    out[3][2] =-(a * t[2] - b * t[4] + c * t[5]);

    t[0] = g * l - k * h; t[1] = f * l - j * h; t[2] = f * k - j * g;
    t[3] = e * l - i * h; t[4] = e * k - i * g; t[5] = e * j - i * f;

    out[0][3] =-(b * t[0] - c * t[1] + d * t[2]);
    out[1][3] =  a * t[0] - c * t[3] + d * t[4];
    out[2][3] =-(a * t[1] - b * t[3] + d * t[5]);
    out[3][3] =  a * t[2] - b * t[4] + c * t[5];

    det = 1.0 / (a * out[0][0] + b * out[1][0]
              +  c * out[2][0] + d * out[3][0]);

    mat4_scale_p(&mut out, det);
    out
}

#[inline]
pub fn mat4_mul(a: Mat4, b: Mat4) -> Mat4 {
    let a00 = a[0][0]; let a01 = a[0][1]; let a02 = a[0][2]; let a03 = a[0][3];
    let a10 = a[1][0]; let a11 = a[1][1]; let a12 = a[1][2]; let a13 = a[1][3];
    let a20 = a[2][0]; let a21 = a[2][1]; let a22 = a[2][2]; let a23 = a[2][3];
    let a30 = a[3][0]; let a31 = a[3][1]; let a32 = a[3][2]; let a33 = a[3][3];

    let b00 = b[0][0]; let b01 = b[0][1]; let b02 = b[0][2]; let b03 = b[0][3];
    let b10 = b[1][0]; let b11 = b[1][1]; let b12 = b[1][2]; let b13 = b[1][3];
    let b20 = b[2][0]; let b21 = b[2][1]; let b22 = b[2][2]; let b23 = b[2][3];
    let b30 = b[3][0]; let b31 = b[3][1]; let b32 = b[3][2]; let b33 = b[3][3];

    let mut dest = mat4_zero();

    dest[0][0] = a00 * b00 + a10 * b01 + a20 * b02 + a30 * b03;
    dest[0][1] = a01 * b00 + a11 * b01 + a21 * b02 + a31 * b03;
    dest[0][2] = a02 * b00 + a12 * b01 + a22 * b02 + a32 * b03;
    dest[0][3] = a03 * b00 + a13 * b01 + a23 * b02 + a33 * b03;
    dest[1][0] = a00 * b10 + a10 * b11 + a20 * b12 + a30 * b13;
    dest[1][1] = a01 * b10 + a11 * b11 + a21 * b12 + a31 * b13;
    dest[1][2] = a02 * b10 + a12 * b11 + a22 * b12 + a32 * b13;
    dest[1][3] = a03 * b10 + a13 * b11 + a23 * b12 + a33 * b13;
    dest[2][0] = a00 * b20 + a10 * b21 + a20 * b22 + a30 * b23;
    dest[2][1] = a01 * b20 + a11 * b21 + a21 * b22 + a31 * b23;
    dest[2][2] = a02 * b20 + a12 * b21 + a22 * b22 + a32 * b23;
    dest[2][3] = a03 * b20 + a13 * b21 + a23 * b22 + a33 * b23;
    dest[3][0] = a00 * b30 + a10 * b31 + a20 * b32 + a30 * b33;
    dest[3][1] = a01 * b30 + a11 * b31 + a21 * b32 + a31 * b33;
    dest[3][2] = a02 * b30 + a12 * b31 + a22 * b32 + a32 * b33;
    dest[3][3] = a03 * b30 + a13 * b31 + a23 * b32 + a33 * b33;

    dest
}