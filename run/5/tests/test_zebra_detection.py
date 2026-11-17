"""Exercise the compiled ARM detector, including real lane extraction."""
from pathlib import Path
from struct import pack, unpack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import *
from test_cross_detection import scene


def machine(process=False, race=False):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mapped = set()
    with (Path(__file__).resolve().parents[1] / 'mdk/Objects/rt1064.axf').open('rb') as f:
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
    base, left, mid, right, stack, stop = 0x30000000, 0x30006000, 0x30006100, 0x30006200, 0x30010000, 0x30020000

    def call(name, args, extra):
        uc.mem_write(stack, pack('<' + 'I' * len(extra), *extra))
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            uc.reg_write(reg, value)
        uc.reg_write(UC_ARM_REG_SP, stack)
        uc.reg_write(UC_ARM_REG_LR, stop | 1)
        uc.emu_start(symbols[name] | 1, stop, count=10000000)
        assert uc.reg_read(UC_ARM_REG_PC) == stop

    def run(frame):
        uc.mem_write(base, bytes(frame))
        call('auxiliary_process', (base, 120, 188, 100), (left, mid, right))
        before = bytes(uc.mem_read(left, 0x300))
        call('zebra_detect', (base, 120, 188, left), (right,))
        assert before == bytes(uc.mem_read(left, 0x300)), 'Lane arrays changed'
        if process:
            call('cross_process', (base, 120, 188, left), (mid, right, 80, 100))
            call('zebra_process', (base, 120, 188, left), (mid, right))
            assert before[:120] == bytes(uc.mem_read(left, 120))
            assert before[512:632] == bytes(uc.mem_read(right, 120))
            run.mid = list(uc.mem_read(mid, 120))
            run.valid = list(uc.mem_read(symbols['img_line_valid'], 120))
            run.draw = list(uc.mem_read(symbols['zebra_draw_flags'], 120))
        assert bytes(uc.mem_read(base, len(frame))) == bytes(frame), 'Image changed'
        if race:
            call('zebra_race_update', (1,), ())
            run.race = tuple(uc.mem_read(symbols[name], 1)[0]
                             for name in ('zebra_pass_count', 'zebra_stop_requested'))
        return list(uc.mem_read(symbols['img_zebra'], 10))
    def motor_target(speed):
        uc.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
        uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
        bits = unpack('<I', pack('<f', speed))[0]
        uc.reg_write(UC_ARM_REG_S0, bits)
        call('zebra_motor_target', (bits,), ())
        return unpack('<f', pack('<I', uc.reg_read(UC_ARM_REG_S0)))[0]

    run.motor_target = motor_target
    return run


def track(top=65, height=10, n=7, width=3, gap=6, lane=100, shift=0, slant=0):
    frame = bytearray(120 * 188)
    for y in range(120):
        for x in range(94 - lane // 2 + 1, 94 + lane // 2):
            frame[y * 188 + x] = 255
    span = n * width + (n - 1) * gap
    for y in range(top, min(120, top + height)):
        start = 94 - span // 2 + shift + round((y - top) * slant)
        for k in range(n):
            for x in range(start + k * (width + gap), start + k * (width + gap) + width):
                if 0 <= x < 188:
                    frame[y * 188 + x] = 0
    return frame


def check():
    for kwargs in ({}, dict(top=35, height=3, width=1, gap=3, lane=48),
                   dict(top=95, height=17, width=4, gap=7, lane=120),
                   dict(n=6), dict(slant=.5)):
        run = machine()
        frame = track(**kwargs)
        a, b, c = run(frame), run(frame), run(frame)
        assert a[0] == 1 and a[1] == b[1] == 0 and c[1] == 1, (kwargs, a, b, c)
        assert c[2] == kwargs.get('n', 7)
        blank = track(height=0)
        for _ in range(4):
            assert run(blank)[1] == 1
        assert run(blank)[1] == 0
    print('PASS: near/far, 1-pixel stripes, six visible stripes, slant, confirmation and release')
    for frame in (track(n=4), track(n=8), track(height=1), track(n=6, height=3),
                  track(height=0), bytes(22560), bytes([255]) * 22560, scene(),
                  track(shift=70)):
        run = machine()
        for _ in range(4):
            result = run(frame)
            assert result[0] == result[1] == 0, result
    run = machine()
    assert run(track())[8] == 1
    assert run(track(height=0))[8] == 0
    assert run(track())[1] == 0
    assert run(track(top=25))[8] == 1, 'Unrelated candidate inherited confirmation'
    print('PASS: wrong counts, one-row noise, short partial band, empty/straight/cross/outside lane rejected; temporal reset')
    for top, height in ((80, 18), (105, 15), (115, 5)):
        run = machine(process=True)
        frame = track(top=top, height=height)
        run(frame)
        assert not any(run.draw), 'Unconfirmed path modified'
        run(frame)
        assert run(frame)[1] == 1
        assert run.mid[top:top+height] == [94] * height, (top, run.mid)
        assert run.valid[top:top+height] == [4] * height
        assert sum(run.draw) == height
        run(track(height=0))
        assert not any(run.draw), 'Stale bridge after candidate disappears'
    # A displaced, sloping lane must use the lane center rather than stripe center.
    run = machine(process=True)
    frame = track(top=80, height=12, shift=5)
    curved = bytearray(len(frame))
    for y in range(120):
        delta = (y - 80) // 4
        for x in range(188):
            if 0 <= x + delta < 188:
                curved[y * 188 + x + delta] = frame[y * 188 + x]
    for _ in range(3):
        result = run(curved)
    assert result[1] == 1
    assert all(abs(run.mid[y] - (94 + (y - 80) // 4)) <= 1 for y in range(80, 92))
    assert sum(run.draw) == 12
    print('PASS: bridge through control region, bottom-clipped exit, sloping/off-center stripes, no stale path')


def check_race():
    run = machine(race=True)
    stripe, clear = track(), track(height=0)
    for _ in range(3):
        run(stripe)
    assert run.race == (1, 0)
    assert run.motor_target(30) == 30
    for _ in range(11):
        run(clear)
    for _ in range(3):
        run(stripe)
    assert run.race == (1, 0), 'Temporary loss must not count the same stripe twice'
    for _ in range(12):
        run(clear)
    for _ in range(3):
        run(stripe)
    assert run.race == (2, 0)
    for _ in range(4):
        run(clear)
        assert run.race == (2, 0)
    run(stripe)
    assert run.race == (2, 0)
    for _ in range(5):
        run(clear)
    assert run.race == (2, 1)
    run(stripe)
    assert run.race == (2, 1), 'Stop request must stay latched'
    speeds = [run.motor_target(30) for _ in range(45)]
    # 目标速度 20 ms 内归零（两拍），实际制动由 10 ms 中断里的反向 PWM 完成。
    assert speeds[0] == 15 and speeds[1] == 0 and all(s == 0 for s in speeds[2:]), speeds[:3]
    assert run.motor_target(30) == run.motor_target(30) == 0
    print('PASS: start crossing, rearm, second crossing, dropout rejection, latched stop')


if __name__ == '__main__':
    check()
    check_race()
