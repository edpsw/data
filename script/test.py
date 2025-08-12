import math
def normalize_angle(q, epsilon=1e-10):
    q_norm = (q + math.pi) % (2 * math.pi) - math.pi
    if abs(q_norm - math.pi) < epsilon:  # 处理 π 的边界
        return -math.pi
    return q_norm

print(normalize_angle(-6))
print(normalize_angle(6))
print(normalize_angle(1e-10))
print(normalize_angle(-1e-10))
print(normalize_angle(-3.14))
print(normalize_angle(3.14))

print(normalize_angle(1e-12))
print(normalize_angle(-1e-12))