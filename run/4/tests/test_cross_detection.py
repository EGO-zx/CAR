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
        uc.emu_start(symbols['cross_detect'] | 1, stop, count=10000000)
        assert uc.reg_read(UC_ARM_REG_PC) == stop, 'Execution did not return'
        assert bytes(uc.mem_read(base, len(frame))) == bytes(frame), 'Input image modified'
        return list(uc.mem_read(symbols['img_cross'], 12))
    return run


def scene(top=40, bottom=70, tilt=0, missing=None, island=False):
    frame=bytearray(120*188)
    for y in range(120):
        for x in range(188):
            shift=round((x-94)*tilt)
            white=60<x<128 or top+shift<y<bottom+shift
            if missing=='lower_left' and x<=60 and y>=bottom+shift: white=True
            if island and x<=60 and y<top+shift-12: white=True
            frame[y*188+x]=255 if white else 0
    return frame

def check():
    run=machine()
    for top,bottom,tilt,island in [(25,45,0,False),(40,70,0,False),(40,75,.18,False),(40,75,-.18,False),(40,75,0,True)]:
        result=run(scene(top,bottom,tilt,island=island))
        assert result[0]==1 and result[3]==15,(top,bottom,tilt,island,result)
        expected=[(60,top+round((60-94)*tilt)),(128,top+round((128-94)*tilt)),(60,bottom+round((60-94)*tilt)),(128,bottom+round((128-94)*tilt))]
        for i,(x,y) in enumerate(expected):
            assert abs(result[4+i]-x)<=3 and abs(result[8+i]-y)<=3,(expected,result)
    print('PASS: early/distant junction, signed unequal corner heights, finite upper-left black region; corner coordinates checked')
    for frame in [bytes(22560),bytes([255])*22560,scene(0,0),scene(missing='lower_left')]:
        assert run(frame)[0]==0
    assert run(scene())[0]==1
    assert run(bytes([255])*22560)[0]==0, 'Result must not latch'
    print('PASS: straight/black/white/missing-corner rejected; per-frame reset; input unchanged')

if __name__=='__main__': check()
