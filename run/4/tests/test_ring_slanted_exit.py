"""Run the exit detector from the freshly built ARM firmware on binary frames."""
from struct import pack, unpack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import *
from test_cross_path import AXF, machine
from test_ring_integration import approach
from test_curve_tracking import track

W, H = 188, 120


def detector():
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mapped = set()
    with AXF.open('rb') as f:
        elf = ELFFile(f)
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        for section in elf.iter_sections():
            if not section['sh_flags'] & 2 or not section['sh_size']:
                continue
            addr, size = section['sh_addr'], section['sh_size']
            for page in range(addr & ~4095, (addr + size + 4095) & ~4095, 4096):
                if page not in mapped:
                    uc.mem_map(page, 4096)
                    mapped.add(page)
            if section['sh_type'] != 'SHT_NOBITS':
                uc.mem_write(addr, section.data())
    base, tracker, result, stack, stop = 0x30000000, 0x30006000, 0x30006100, 0x30010000, 0x30020000
    uc.mem_map(base, 0x30000)

    def run(frame, state=3):
        uc.mem_write(base, bytes(frame))
        uc.mem_write(stack, pack('<IIIII', W, 85, H-1, result, 0))
        for reg, val in ((UC_ARM_REG_R0, tracker), (UC_ARM_REG_R1, state),
                         (UC_ARM_REG_R2, base), (UC_ARM_REG_R3, H),
                         (UC_ARM_REG_SP, stack), (UC_ARM_REG_LR, stop | 1)):
            uc.reg_write(reg, val)
        uc.emu_start(symbols['ring_s3_exit_update'] | 1, stop, count=10000000)
        assert uc.reg_read(UC_ARM_REG_PC) == stop
        data = bytes(uc.mem_read(result, 10))
        return data[2], data[1], unpack('<hhh', data[4:10])
    return run


def road(edge, left=35):
    frame = bytearray(W*H)
    for y in range(H):
        r = min(W-1, edge(y))
        frame[y*W+left:y*W+r+1] = bytes([255])*(r-left+1)
    return frame


def check():
    # One-sided, diagonal opening: no left arm and no black top cap.
    slanted = road(lambda y: 112 + abs(y-62))
    run = detector()
    for expected in (0, 0, 1):
        confirmed, valid, (x, y, bar) = run(slanted)
        assert confirmed == expected and valid
        assert abs(x-113) <= 1 and abs(y-62) <= 1 and bar == -1

    # Smooth curves, straight/monotonic edges, isolated spikes and frame edges.
    for edge in (lambda y: 150, lambda y: 100+y//3,
                 lambda y: 112+(y-62)**2//160,
                 lambda y: 112 if y == 62 else 150,
                 lambda y: W-1):
        run = detector()
        for _ in range(5):
            assert run(road(edge))[0] == 0

    run = detector()
    assert not run(slanted)[0]
    assert not run(slanted)[0]
    assert not run(bytes(W*H))[0]
    for expected in (0, 0, 1):
        assert run(slanted)[0] == expected
    assert not run(slanted, state=2)[0]
    assert not run(slanted)[0]

    # Detached bright patch cannot supply a turn to the driving corridor.
    frame = road(lambda y: 150)
    frame[80*W:83*W] = bytes(3*W)
    frame[:80*W] = slanted[:80*W]
    run = detector()
    for _ in range(4):
        assert not run(frame)[0]

    # Enter S3 through the public pipeline, then exit through a diagonal corner.
    # A bounded curved left component must be reacquired without a T bar row.
    firmware = machine(integrated=True)
    normal = track(lambda y: 94)
    bend = track(lambda y: 94+(y-90)**2//400)
    for _ in range(4): firmware(normal)
    firmware(approach())
    firmware(approach(34,109,None))
    for _ in range(3): firmware(approach(42,None,None,True))
    for _ in range(3): firmware(bend)
    assert firmware.ring == 3 and not firmware.island
    frame = road(lambda y: 112+abs(y-62), left=0)
    frame[:30*W] = bytes(30*W)  # Measured upper boundary for S4's left anchor.
    for y in range(72,H):
        left = 46+(y-92)**2//80
        frame[y*W:y*W+left] = bytes(left)
    for expected in (3,3,4):
        firmware(frame)
        assert firmware.ring == expected
    assert firmware.island, 'Diagonal exit did not reacquire inner island'
    # S4 must recover after the component tracker exhausts its lost-frame limit.
    for _ in range(3):
        firmware(bytes(W*H))
        assert firmware.ring == 4 and not firmware.island
    for _ in range(3):
        _, _, _, valid, _ = firmware(frame)
        assert firmware.ring == 4
    assert firmware.island, 'S4 never re-seeded after losing the island'
    assert sum(bool(valid[y]) for y in range(80,101)) >= 15
    for expected in (4,5):
        firmware(approach(30,None,None,True))
        assert firmware.ring == expected
    for expected in (5,5,0):
        firmware(normal)
        assert firmware.ring == expected
    print('PASS: diagonal exit, 3-frame confirmation, dropout/state reset, '
          'smooth-bend/spike/frame-edge rejection, disconnected patch, '
          'full diagonal ring cycle and island reacquisition')


if __name__ == '__main__':
    check()
