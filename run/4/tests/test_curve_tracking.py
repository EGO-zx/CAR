"""Run the actual Keil-built ARM image-processing function on synthetic frames.
Requires: pip install unicorn pyelftools. Build run/3/mdk/rt1064.uvprojx first.
"""
from pathlib import Path
from struct import pack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import *

AXF = Path(__file__).resolve().parents[1] / 'mdk/Objects/rt1064.axf'

def machine(axf=AXF):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mapped = set()
    with Path(axf).open('rb') as f:
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
    uc.mem_map(0x30000000, 0x30000)
    def run(frame, threshold=100):
        base, left, mid, right, stack, stop = 0x30000000, 0x30006000, 0x30006100, 0x30006200, 0x30010000, 0x30020000
        uc.mem_write(base, bytes(frame))
        uc.mem_write(left, bytes([199]) * 0x300)
        uc.mem_write(stack, pack('<III', left, mid, right))
        for reg, value in ((UC_ARM_REG_R0, base), (UC_ARM_REG_R1, 120),
                           (UC_ARM_REG_R2, 188), (UC_ARM_REG_R3, threshold),
                           (UC_ARM_REG_SP, stack), (UC_ARM_REG_LR, stop | 1)):
            uc.reg_write(reg, value)
        uc.emu_start(symbols['auxiliary_process'] | 1, stop, count=10000000)
        assert uc.reg_read(UC_ARM_REG_PC) == stop, 'Execution did not return'
        assert bytes(uc.mem_read(base, len(frame))) == bytes(frame), 'Input image modified'
        for address in (left, mid, right):
            assert bytes(uc.mem_read(address + 120, 136)) == bytes([199]) * 136, 'Output overflow'
        return tuple(list(uc.mem_read(a, 120)) for a in
                     (left, mid, right, symbols['img_line_valid'], symbols['img_edge_valid']))
    return run

def track(center, width=lambda row: 100):
    data = bytearray(120 * 188)
    for row in range(120):
        c = center(row)
        half = width(row) // 2
        # Known synthetic boundaries, independent of the reconstruction code.
        for x in range(188):
            if c - half < x < c + half:
                data[row * 188 + x] = 255
    return data

def run_regression():
    run = machine()
    straight = track(lambda r: 94)
    for threshold in (0, 100, 255):
        l, m, r, valid, edges = run(straight, threshold)
        assert m == [94] * 120 and valid == [2] * 120 and edges == [3] * 120
    print('PASS: straight track, including thresholds 0 and 255')
    for direction in (-1, 1):
        run = machine()
        run(straight)
        def curve(row):
            return 94 + direction * min(70, (119 - row) * 2)
        l, m, r, valid, edges = run(track(curve))
        assert all(abs(m[row] - curve(row)) <= 1 for row in range(120))
        assert valid[0] == 1 and valid[119] == 2
        print('PASS: clipped', 'left' if direction < 0 else 'right', 'bend; center error <= 1 pixel')
    for color in (0, 255):
        l, m, r, valid, edges = run(bytes([color]) * (120 * 188))
        assert not any(valid) and not any(edges)
        assert all(0 <= x < 188 for x in l + m + r)
    print('PASS: all-black/all-white frames invalid, coordinates bounded')
    run = machine()
    l, m, r, valid, edges = run(track(lambda r: 24))
    assert not any(valid), 'Uncalibrated single-edge image must not invent a center'
    print('PASS: no fabricated center before width calibration')
    for center in (24, 164):
        run = machine()
        run(straight)
        for frame_number in range(300):
            l, m, r, valid, edges = run(track(lambda r: center))
            assert valid == [1] * 120 and m == [center] * 120, frame_number
        # Complete loss must still be invalid, but must not erase learned width.
        for color in (0, 255):
            l, m, r, valid, edges = run(bytes([color]) * (120 * 188))
            assert not any(valid)
        l, m, r, valid, edges = run(track(lambda r: center))
        assert valid == [1] * 120 and m == [center] * 120
        # Reliable dual edges can still update width after prolonged single-edge use.
        wider = bytearray(120 * 188)
        for row in range(120):
            for x in range(35, 154):
                wider[row * 188 + x] = 255
        run(wider)
        # The actual lane remains wider when its left edge leaves the frame.
        l, m, r, valid, edges = run(track(lambda r: 24, lambda r: 120))
        assert valid == [1] * 120 and m == [24] * 120
    print('PASS: both single-edge directions remain valid for 300 frames; width survives loss and updates')
    run = machine()
    run(straight)
    frame = track(lambda r: 94 - min(70, (119-r)*2))
    frame[85*188:88*188] = bytes(3*188)
    l, m, r, valid, edges = run(frame)
    assert valid[85:88] == [0,0,0] and valid[84] != 0
    print('PASS: short missing band remains invalid and tracking recovers')
    
    # A missing steering row does not imply the rest of the image is invalid.
    run = machine()
    run(straight)
    frame = bytearray(straight)
    frame[90*188:91*188] = bytes(188)
    l, m, r, valid, edges = run(frame)
    assert valid[90] == 0 and sum(bool(v) for v in valid) == 119
    print('PASS: missing row 90 is invalid while the other 119 rows remain valid')
    
    # Seeded noise: validate bounds and flag consistency, not real-track accuracy.
    import random
    rng = random.Random(20261018)
    run = machine()
    run(straight)
    for _ in range(100):
        frame = bytearray(straight)
        for _ in range(200):
            index = rng.randrange(len(frame))
            frame[index] ^= 255
        l, m, r, valid, edges = run(frame)
        assert all(0 <= x < 188 for x in l + m + r)
        assert all(v in (0, 1, 2) for v in valid)
        assert all(e in (0, 1, 2, 3) for e in edges)
        assert all(v == 0 or (v == 1 and e in (1,2)) or
                   (v == 2 and e == 3) for v, e in zip(valid, edges))
    print('PASS: 100 noisy frames; bounded coordinates, consistent flags, intact input/output guards')


def run_variable_width_tests():
    from math import sqrt
    baseline = Path(__file__).resolve().parents[2] / '2/tests/before_anchor_strategy/rt1064.axf'
    straight = track(lambda r: 94)
    for direction in (-1, 1):
        center = lambda r: 94 + direction * min(70, 2 * (119-r))
        width = lambda r: 100 + min(70, 119-r)
        frame = track(center, width)
        old, new = machine(baseline), machine()
        old(straight)
        new(straight)
        _, old_mid, _, old_valid, _ = old(frame)
        _, new_mid, _, new_valid, _ = new(frame)
        roi = range(80, 101)
        assert all(new_valid[r] for r in roi)
        before = sum(abs(old_mid[r]-center(r)) for r in roi) / len(roi)
        after = sum(abs(new_mid[r]-center(r)) for r in roi) / len(roi)
        assert max(abs(new_mid[r]-center(r)) for r in roi) <= 2
        assert after < before / 3
        print(f'PASS: variable-width bend direction {direction}: ROI error {before:.2f} -> {after:.2f} pixels')
        # Remove all dual-edge anchors without changing the lane width profile.
        # The current bend model must persist, rather than reverting to straight width.
        single_center = 24 if direction == -1 else 164
        single = track(lambda r: single_center, width)
        for _ in range(300):
            _, mid, _, valid, _ = new(single)
            assert valid[90] == 1 and abs(mid[90]-single_center) <= 2
        print('PASS: learned bend model survives 300 entirely single-edge frames')

    # An image-plane sloping corridor: its horizontal span grows with tangent.
    # This is an independent geometry test, not a calibrated camera simulation.
    for direction in (-1, 1):
        center = lambda r: round(94 + direction * 0.04 * (119-r)**2)
        width = lambda r: round(60 * sqrt(1 + (0.08 * (119-r))**2))
        old, new = machine(baseline), machine()
        calibration = track(lambda r: 94, lambda r: 60)
        old(calibration)
        new(calibration)
        frame = track(center, width)
        _, om, _, ov, _ = old(frame)
        _, nm, _, nv, _ = new(frame)
        roi = range(80, 91)
        assert all(nv[r] for r in roi)
        before = sum(abs(om[r]-center(r)) for r in roi) / len(roi)
        after = sum(abs(nm[r]-center(r)) for r in roi) / len(roi)
        assert after < before / 3
        print(f'PASS: curved corridor direction {direction}: ROI error {before:.2f} -> {after:.2f} pixels')


if __name__ == '__main__':
    run_regression()
    run_variable_width_tests()
