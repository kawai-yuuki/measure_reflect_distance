import math
import numpy as np

# =========================
# 線形代数の最小ユーティリティ（依存削減のため自前実装）
# =========================

def rot_from_rpy(roll, pitch, yaw):
    """
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    - ROSのRPY規約に合わせて Z(ヨー)→Y(ピッチ)→X(ロール) の順
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0,  cr, -sr],
                   [0.0,  sr,  cr]])
    return Rz @ Ry @ Rx

def rot_from_quat(x, y, z, w):
    """
    クォータニオン (x, y, z, w) から回転行列 3x3 を作る。
    - 正規化を内部で行う
    - 標準的な式に基づく実装
    """
    n = x*x + y*y + z*z + w*w
    if n < 1e-16:
        return np.eye(3)
    s = 2.0 / n
    X, Y, Z = x*s, y*s, z*s
    wx, wy, wz = w*X, w*Y, w*Z
    xx, xy, xz = x*X, x*Y, x*Z
    yy, yz, zz = y*Y, y*Z, z*Z
    return np.array([
        [1.0-(yy+zz),   xy-wz,       xz+wy],
        [  xy+wz,     1.0-(xx+zz),   yz-wx],
        [  xz-wy,       yz+wx,     1.0-(xx+yy)]
    ])

def quat_from_R(R):
    """回転行列(3x3) → クォータニオン(x,y,z,w)。安定な分岐版。"""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
            w = (R[2,1] - R[1,2]) / s
        elif i == 1:
            s = math.sqrt(1.0 - R[0,0] + R[1,1] - R[2,2]) * 2.0
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
            w = (R[0,2] - R[2,0]) / s
        else:
            s = math.sqrt(1.0 - R[0,0] - R[1,1] + R[2,2]) * 2.0
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
            w = (R[1,0] - R[0,1]) / s
    q = np.array([x, y, z, w], dtype=float)
    q /= (np.linalg.norm(q) + 1e-12)
    return q

def quat_from_two_vectors(a, b):
    """
    単位ベクトル a を b に最短回転で合わせるクォータニオン (x,y,z,w) を返す。
    - 境界ケース: a と b がほぼ反対のときは任意の直交軸で 180 度回転
    - 可視化用の姿勢生成（Z軸→法線）に使用
    """
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    w = 1.0 + float(a @ b)
    if w < 1e-8:
        # 180度回転：aに直交する任意軸を使う
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / (np.linalg.norm(v) + 1e-12)
        return np.array([v[0], v[1], v[2], 0.0])
    q = np.array([v[0], v[1], v[2], w])
    return q / (np.linalg.norm(q) + 1e-12)

def mat4_from_rt(R, t):
    """回転 R(3x3) と 並進 t(3,) から 4x4 同次変換を作る。"""
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = t
    return T

def mat4_from_tf(translation, quat_xyzw):
    """geometry_msgs/Transform の値から 4x4 同次変換を作る。"""
    x, y, z = translation
    qx, qy, qz, qw = quat_xyzw
    R = rot_from_quat(qx, qy, qz, qw)
    return mat4_from_rt(R, np.array([x, y, z], dtype=float))

def inv_T(T):
    """同次変換の逆行列（剛体変換）"""
    R = T[:3,:3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def plane_from_reflection(S):
    """
    反射変換 S から平面パラメータ (n, d) を復元。
    - S = [[R, t],[0,1]],  R = I - 2 n n^T,  t = 2 d n
    - M = (I - R)/2 = n n^T は rank-1。SVD の最大特異ベクトルが n。
    - d = 0.5 * n^T t
    - 規約: カメラ前方の平面で d < 0 になるよう符号を揃える（n はカメラに向く向き）
    """
    R = S[:3,:3]
    t = S[:3, 3]
    M = 0.5 * (np.eye(3) - R)
    U, s, Vt = np.linalg.svd(M)
    idx = int(np.argmax(s))
    n = U[:, idx]
    n = n / (np.linalg.norm(n) + 1e-12)
    d = 0.5 * float(n @ t)
    if (-d * n[2]) < 0.0:  # カメラ前方に法線が向くように
        n = -n
        d = -d
    return n, d

def transform_plane(N_a, T_b_a):
    """
    平面4ベクトル N の座標変換（論文と同じ転置ルール）:
        N_b = (T_b_a^{-1})^T * N_a
    - N = [nx, ny, nz, d]^T,  平面方程式は n^T x + d = 0
    - 最後に ||n||=1 へ正規化
    """
    T_inv = inv_T(T_b_a)
    N_b = T_inv.T @ N_a
    n = N_b[:3]
    d = N_b[3]
    norm = np.linalg.norm(n) + 1e-12
    n /= norm
    d /= norm
    return np.array([n[0], n[1], n[2], d])
