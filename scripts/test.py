import transform as tf

def test(a, b):
    if a==b:
        print("PASS")
    else:
        print("FAIL")
    return



t1 = tf.Transform(0,0,0,0,1,0,0)
t2 = tf.Transform(0,0,0,0,0,1,0)
t3 = t1*t2
test(t3,t3)

print(t3.axisangle())
print(t3)
