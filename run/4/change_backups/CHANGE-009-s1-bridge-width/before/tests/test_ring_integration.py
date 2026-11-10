"""Exercise the linked run/4 firmware on binary frames, without forcing any state."""
from test_cross_path import machine, scene
from test_curve_tracking import track

W, H = 188, 120

def approach(upper=30, arc=62, lower=103, triangle=False):
    frame = bytearray([255]) * (W * H)
    for y in range(H):
        frame[y*W+160:(y+1)*W] = bytes(28)
        edge = 70 if y <= upper else -1
        if arc is not None and abs(y-arc) <= 8:
            edge = 60-abs(y-arc)
        if lower is not None and y >= lower:
            edge = 32
        if triangle and y >= 62:
            edge = y-62
        if edge >= 0:
            frame[y*W:y*W+edge+1] = bytes(edge+1)
    return frame

def junction(curved=True):
    frame = bytearray(W*H)
    for y in range(42,H):
        left, right = (0,W) if y<68 else (46+((y-92)**2//80 if curved else 0),140)
        frame[y*W+left:y*W+right] = bytes([255])*(right-left)
    return frame

def drive(run, bottom_exit=False, curved=True):
    normal = track(lambda y:94)
    bend = track(lambda y:94+(y-90)**2//400)
    def step(image, expected, tag=None):
        left,mid,right,valid,edges = run(image)
        assert run.ring == expected, (expected,run.ring)
        if expected: assert run.state==0, "cross reconstruction active inside ring"
        if tag is not None:
            assert valid[90]==tag,(expected,valid[90],tag)
        if expected==3:
            for y in range(H):
                if edges[y]==3 and run.base_valid[y]==2: assert mid[y]==run.base_mid[y] and valid[y]==2
        if expected==0:
            assert bytes(mid)==run.base_mid and bytes(valid)==run.base_valid
        return mid,valid
    for _ in range(4):step(normal,0,2)
    step(approach(),1,4)
    step(approach(34,99 if bottom_exit else 109,None),1,4)
    assert run.island
    for state in (1,2,2):
        step(approach(42,None,None,True),state,5 if state==2 else 4)
    for state in (2,2,3):
        step(bend,state)
    assert not run.island
    for state in (3,3,4):
        frame=junction(curved)
        if bottom_exit:
            # Far right shoulder at row 68; left shoulder only visible near bottom.
            for y in range(68,99):
                frame[y*W:y*W+100]=bytes([255])*100
        mid,valid=step(frame,state)
    assert bool(run.island)==curved
    # S4 uses the bridge and connected right shoulder, independent of the island.
    assert all(valid[y]==6 and mid[y]==90 for y in range(80,111))
    # A notch in a detached bright patch has the local upper-corner polarity.
    # Repeated sightings outside the connected driving corridor must not enter S5.
    for y in range(4,25):
        for x in range(40,92):
            if y>14 or x>68:frame[y*W+x]=255
    for _ in range(4):step(frame,4)
    for state in (4,5):
        step(approach(30,None,None,True),state)
    if bottom_exit:
        for y in range(40,111,10):
            step(approach(y,None,None),5 if y<108 else 0)
    else:
        for state in (5,5,0):step(normal,state,2 if not state else 4)

def check():
    for curved,expected in [(True,1),(False,0)]:
        trial=machine(integrated=True)
        frame=approach(upper=-1,arc=None)
        for y in range(44,51):
            end=60-abs(y-47) if curved else 60
            frame[y*W:y*W+end+1]=bytes(end+1)
        trial(frame)
        assert trial.ring==expected, ("small distant island",curved,trial.ring)

    for image,expected in [(approach(upper=-1),1),(approach(upper=-1,lower=None),0),
                           (approach(upper=-1,arc=None),0)]:
        trial=machine(integrated=True)
        trial(image)
        assert trial.ring==expected, "optional upper; mandatory lower and middle"

    for gaps,expected in [((71,72,94),1),((71,72,73),0),((71,72,94,98),0)]:
        trial=machine(integrated=True)
        frame=approach()
        for y in gaps:frame[y*W+160:(y+1)*W]=bytes([255])*(W-160)
        *_,edges=trial(frame)
        assert all(not edges[y]&2 for y in gaps)
        assert trial.ring==expected,(gaps,trial.ring)

    for samples in (0,3,4):
        cold=machine(integrated=True)
        for _ in range(samples):cold(track(lambda y:94))
        _,mid,right,valid,_=cold(approach())
        assert cold.ring==1
        if samples==4:
            assert valid[90]==4 and mid[90]==right[90]-50
        else:
            assert not any(valid), "unfinished calibration used for ring steering"

    run=machine(integrated=True)
    probe=machine(integrated=True)
    for _ in range(4):probe(track(lambda y:94))
    probe(approach())
    for lower,expected in [(None,1),(103,1),(None,1),(None,1)]:
        probe(approach(-1,62,lower))
        assert probe.ring==expected,(probe.ring,expected)
    # Lower corner missing alone, or all-black without a confirmed island, stays in S1.
    probe(bytes(W*H))
    assert probe.ring==1
    drive(run)
    drive(run,bottom_exit=True)
    drive(run,curved=False)
    run(scene(40,95))
    assert run.state==1 and run.ring==0
    run(approach())
    assert run.ring==0, "active cross preempted by ring"
    print("PASS: 3 complete ring cycles, both exits, same-frame midline ownership, "
          "island loss/reacquisition and rejection, active-cross priority")

if __name__=='__main__':check()
