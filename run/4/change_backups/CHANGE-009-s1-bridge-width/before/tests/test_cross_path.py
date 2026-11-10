"""Run the actual Keil-built ARM image-processing function on synthetic frames.
Requires: pip install unicorn pyelftools. Build run/3/mdk/rt1064.uvprojx first.
"""
from pathlib import Path
from struct import pack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import *

AXF = Path(__file__).resolve().parents[1] / 'mdk/Objects/rt1064.axf'

def machine(axf=AXF, *, integrated=False):
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
        before=bytes(uc.mem_read(left,120))+bytes(uc.mem_read(right,120))
        uc.mem_write(stack,pack('<IIII',mid,right,80,100))
        for reg,value in ((UC_ARM_REG_R0,base),(UC_ARM_REG_R1,120),(UC_ARM_REG_R2,188),
                          (UC_ARM_REG_R3,left),(UC_ARM_REG_SP,stack),(UC_ARM_REG_LR,stop|1)):
            uc.reg_write(reg,value)
        run.base_mid = bytes(uc.mem_read(mid,120))
        run.base_valid = bytes(uc.mem_read(symbols['img_line_valid'],120))
        target = 'track_special_process' if integrated else 'cross_process'
        uc.emu_start(symbols[target]|1,stop,count=120000000)
        if integrated:
            run.ring = uc.mem_read(symbols['ring'],1)[0]
            run.island = uc.mem_read(symbols['island'],1)[0]
        assert uc.reg_read(UC_ARM_REG_PC)==stop
        assert before==bytes(uc.mem_read(left,120))+bytes(uc.mem_read(right,120)), 'Raw edges changed'
        run.state=uc.mem_read(symbols['cross_path_state'],1)[0]
        run.mask=uc.mem_read(symbols['img_cross']+3,1)[0]
        run.corners=list(uc.mem_read(symbols['img_cross']+4,8))
        run.draw=[list(uc.mem_read(symbols[name],120)) for name in ('cross_draw_left','cross_draw_right','cross_draw_flags')]

        assert bytes(uc.mem_read(base, len(frame))) == bytes(frame), 'Input image modified'
        for address in (left, mid, right):
            assert bytes(uc.mem_read(address + 120, 136)) == bytes([199]) * 136, 'Output overflow'
        return tuple(list(uc.mem_read(a, 120)) for a in
                     (left, mid, right, symbols['img_line_valid'], symbols['img_edge_valid']))
    return run


from test_cross_detection import scene

def check():
    run=machine()
    result=run(scene(25,45))
    assert run.state==1 and all(result[3][y]==2 for y in range(80,101))
    run(scene(35,65))
    run(scene(40,85))
    result=run(scene(40,95))
    assert run.state==1 and all(result[3][y]==3 for y in range(45,95))
    result=run(scene(40,95,missing='lower_left'))
    assert run.state==3,(run.state,run.mask)
    result=run(scene(40,140))
    assert run.state==2,(run.state,run.mask)
    assert all(result[3][y]==3 and abs(result[1][y]-94)<=2 for y in range(80,120))
    # Move current exit sideways: reconstruction must follow new pixels, not old line.
    f=scene(40,140)
    shifted=bytearray(len(f))
    for y in range(120):
        for x in range(188): shifted[y*188+x]=f[y*188+max(0,x-6)]
    result=run(shifted)
    assert run.state==2 and all(abs(result[1][y]-100)<=2 for y in range(80,101))
    assert all(run.draw[2][y]==3 for y in range(80,101))
    assert all(result[1][y]==(run.draw[0][y]+run.draw[1][y])//2 for y in range(80,101))
    result=run(scene(0,0))
    assert run.state==0 and all(result[3][y]==2 for y in range(80,120))
    fresh=machine(); result=fresh(scene(40,140))
    assert fresh.state==0, 'Two corners cannot start passage'
    print('PASS: four-corner entry, unchanged near lane, mixed side, current-frame exit fitting, display/control agreement, recovery, startup gate')

def check_corner_tracking(integrated=False):
    run=machine(integrated=integrated)
    run(scene(40,95))
    run(scene(40,140))
    # A wrong narrow exit on the right has genuine L-shaped ends, but is not
    # the exit established by the preceding four-corner frame.
    narrow=bytearray(120*188)
    for y in range(120):
        for x in range(188):
            narrow[y*188+x]=255 if y>40 or 100<x<120 else 0
    result=run(narrow)
    assert run.state==4 and run.mask&3==0,(run.state,run.mask,run.corners)
    assert not any(run.draw[2]), 'Rejected pair still drawn'
    assert all(result[3][y]==0 for y in range(80,101))
    # A rejected frame must not replace the last accepted corner reference.
    result=run(scene(40,140))
    assert run.state==2 and abs(run.corners[0]-60)<=2 and abs(run.corners[1]-128)<=2
    # The upper pair can move and have different heights; no fixed coordinates.
    for shift in (3,6,9):
        source=scene(43,140,tilt=.12)
        frame=bytearray(120*188)
        for y in range(120):
            for x in range(188): frame[y*188+x]=source[y*188+max(0,x-shift)]
        run(frame)
        assert run.state==2,(run.state,run.mask,run.corners)
        assert abs(run.corners[0]-(60+shift))<=3
        assert abs(run.corners[1]-(128+shift))<=3
    run(scene(0,0))
    assert run.state==0
    run(scene(25,45))
    assert run.state==1, 'New junction must not inherit old corner tracking'
    print('PASS: right-shifted narrow false pair rejected, reacquisition, moving staggered pair, exit reset')

def check_lower_departure():
    for integrated in (False,True):
        run=machine(integrated=integrated)
        run(scene(40,95))
        run(scene(40,112))
        run(scene(40,140))
        fake=scene(40,140)
        for y in range(104,120):
            for x in range(91):fake[y*188+x]=0
        for frame in (fake,scene(40,140),fake,scene(40,112)):
            result=run(frame)
            assert run.state==2 and run.mask&12==0,(run.state,run.mask,run.corners)
            assert all(result[3][y]==3 and abs(result[1][y]-94)<=2 for y in range(80,101))
        run(scene(0,0))
        run(scene(40,95))
        assert run.state==1 and run.mask==15, 'Departure flags leaked into next junction'
        # Away from bottom, a transient loss may recover its original lower point.
        run(scene(40,95,missing='lower_left'))
        assert run.state==3
        run(scene(40,95))
        assert run.state==1
        # Each side leaves view independently.
        run(scene(40,112))
        run(scene(40,112,missing='lower_left'))
        assert run.state==3 and run.mask&4==0 and run.mask&8
        run(scene(40,112))
        assert run.state==3 and run.mask&4==0
    print('PASS: departed lower corners cannot reattach; no repeated 10px steering jump; independent sides, transient loss, next-junction reset')

def check_upper_identity():
    for integrated in (False,True):
        run=machine(integrated=integrated)
        run(scene(40,95));run(scene(40,112))
        # True right junction shoulder shifts right. A closer bend corner
        # appears 10 rows above it and wins the old position-only score.
        f=scene(40,140)
        for y in range(41):
            for x in range(94,188):
                edge=140 if y>30 else 128
                f[y*188+x]=255 if x<edge else 0
        for _ in range(4):
            run(f)
            assert run.mask&3==3 and run.state==2,(run.state,run.corners)
            assert abs(run.corners[1]-140)<=3 and run.corners[5]==40,run.corners
        # The two junction corners need not be at the same height.
        run=machine(integrated=integrated)
        run(scene(40,95,tilt=.18));run(scene(40,112,tilt=.18))
        run(scene(40,140,tilt=.18))
        assert run.state==2 and run.corners[5]!=run.corners[4]
    print('PASS: original shoulder identity wins over nearer bend corner; no progressive migration; staggered shoulders supported')

if __name__=='__main__':
    check()
    check_corner_tracking()
    check_corner_tracking(integrated=True)
    check_lower_departure()
    check_upper_identity()

