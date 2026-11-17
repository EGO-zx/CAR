"""S4 bridge tests against the linked firmware; no forced state or synthetic edges."""
from test_cross_path import machine
from test_ring_integration import approach
from test_curve_tracking import track

W, H = 188, 120


def exit_frame():
    frame = bytearray(W*H)
    for y in range(H):
        for x in range(W):
            # Sloping upper border, pointed right shoulder and only seven rows
            # of left island: too short for the original island detector.
            if y > 50-x//20 and x <= 110+abs(y-82):
                if not (y >= 113 and x <= 10):
                    frame[y*W+x] = 255
    return frame


def enter_four(frame):
    run = machine(integrated=True)
    for _ in range(4): run(track(lambda y:94))
    run(approach())
    run(approach(34,109,None))
    for _ in range(3): run(approach(42,None,None,True))
    for _ in range(3): run(track(lambda y:94+(y-90)**2//400))
    assert run.ring == 3
    for expected in (3,3,4):
        result = run(frame)
        assert run.ring == expected
    return run, result


def check():
    frame = exit_frame()
    run, (_,mid,right,valid,_) = enter_four(frame)
    assert not run.island, 'Fixture unexpectedly has a usable inner island'
    assert sum(v==6 for v in valid[80:101]) >= 18
    assert any(valid[y]==6 for y in range(60,82)), 'Bridge segment missing'
    for y in range(H):
        if valid[y]:
            assert frame[y*W+mid[y]], 'Midline entered black region'
            if y>82: assert mid[y]==right[y]-50
    # Repeated frames must continue producing the same S4 guidance.
    for _ in range(3):
        _,again,_,flags,_ = run(frame)
        assert run.ring == 4 and again==mid and flags==valid

    # No measured black cap in the five anchor columns: no invented endpoint.
    missing = bytearray(frame)
    for y in range(54):
        missing[y*W+16:y*W+21] = bytes([255])*5
    _, (_,_,_,flags,_) = enter_four(missing)
    assert not any(flags)

    # One noisy column does not destroy the neighboring-column consensus.
    noisy = bytearray(frame)
    for y in range(54): noisy[y*W+16] = 255
    _, (_,_,_,flags,_) = enter_four(noisy)
    assert sum(v==6 for v in flags[80:101]) >= 18

    # An obstacle on the proposed bridge must reject that bridge.
    blocked = bytearray(frame)
    for y in range(62,68):
        blocked[y*W+57:y*W+64] = bytes(7)
    _, (_,_,_,flags,_) = enter_four(blocked)
    assert not any(flags), 'Bridge crossed a black obstacle'
    print('PASS: S4 bridge with short island, control-window coverage, '
          'steady tracking, missing-anchor rejection, noisy-column tolerance, '
          'black-obstacle rejection and input preservation')


if __name__ == '__main__':
    check()
