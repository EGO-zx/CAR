"""S1 measured-tip and straight-only guidance checks on compiled ARM firmware."""
from struct import unpack
from test_cross_path import machine
from test_ring_integration import approach
from test_curve_tracking import track

W,H=188,120


def entered():
    run=machine(integrated=True)
    for _ in range(4):run(track(lambda y:94))
    run(approach())
    assert run.ring==1
    return run


def middle(run):
    return unpack('<hhB',run.marks[6:11])


def right_shape(edge, gaps=()):
    frame=approach()
    for y in range(H):
        frame[y*W+90:(y+1)*W]=bytes([255])*(W-90)
        if y not in gaps:
            x=edge(y)
            frame[y*W+x:(y+1)*W]=bytes(W-x)
    return frame


def island(edge, first, last):
    frame=approach(-1,None,None)
    for y in range(first,last+1):
        x=edge(y)
        frame[y*W:y*W+x+1]=bytes(x+1)
    return frame


def check():
    run=entered()
    _,mid,right,valid,_=run(right_shape(lambda y:150+(y-100)//4))
    assert run.ring==1 and all(valid[y]==4 for y in range(80,101))
    assert all(mid[y]==right[y]-50 for y in range(80,101))

    run=entered()
    _,_,_,valid,_=run(right_shape(lambda y:160 if y>=65 else max(95,160-(65-y)**2//12)))
    assert all(valid[y]==4 for y in range(80,101))
    assert not any(valid[:50]), 'S1 followed far curved branch'

    run=entered()
    _,_,_,valid,_=run(right_shape(lambda y:160,gaps=(60,61,62)))
    assert not any(valid[:63]), 'S1 restarted above a disconnected fragment'
    assert all(valid[y]==4 for y in range(80,101))

    run=entered()
    _,_,_,valid,_=run(right_shape(lambda y:160,gaps=(61,)))
    assert not valid[61] and valid[60]==4 and valid[62]==4

    # True rounded tip: both sides of the actual boundary support the peak.
    run=entered()
    frame=island(lambda y:70-abs(y-70),55,85)
    run(frame)
    x,y,ok=middle(run)
    assert ok and (x,y)==(70,70), (x,y,ok)
    assert frame[y*W+x]==0 and frame[y*W+x+1]!=0

    # Tall near island with a rounded top shoulder and a long monotonic flank.
    # A short local peak is not the in-island arc tip.
    run=entered()
    frame=island(lambda y:65-abs(y-39) if y<=43 else 61-(y-43)//2,33,105)
    run(frame)
    assert not middle(run)[2], ('top shoulder accepted',middle(run))
    # After losing the real arc, never retain its old purple marker.
    run=entered()
    run(island(lambda y:70-abs(y-70),55,85))
    run(frame)
    assert not middle(run)[2]
    print('PASS: inclined straight, far bend cutoff, disconnected fragment, '
          'single-row gap, measured arc tip, near top-shoulder rejection, no stale marker')


if __name__=='__main__':check()
