"""Check strict upper/middle order at the firmware entry API."""
from struct import pack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import *
from test_cross_path import AXF

def check():
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
    mapped=set()
    with AXF.open('rb') as f:
        elf=ELFFile(f)
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        for section in elf.iter_sections():
            if not section['sh_flags']&2 or not section['sh_size']:continue
            addr,size=section['sh_addr'],section['sh_size']
            for page in range(addr&~4095,(addr+size+4095)&~4095,4096):
                if page not in mapped:uc.mem_map(page,4096);mapped.add(page)
            if section['sh_type']!='SHT_NOBITS':uc.mem_write(addr,section.data())
    base=0x30000000
    uc.mem_map(base,0x30000)
    def point(x,y,valid=1):return pack('<hhBx',x,y,valid)
    for upper_y,upper_valid,expected in [(30,1,1),(62,1,0),(70,1,0),(30,0,1)]:
        uc.mem_write(base,bytes(64))
        # confirmed deliberately set: the entry API must independently reject bad ordering.
        observation=bytes([1,1,0,0])+point(32,103)+point(60,62)+point(70,upper_y,upper_valid)
        uc.mem_write(base+256,observation)
        for reg,val in [(UC_ARM_REG_R0,base),(UC_ARM_REG_R1,120),(UC_ARM_REG_R2,188),
                        (UC_ARM_REG_R3,base+256),(UC_ARM_REG_SP,base+0x10000),
                        (UC_ARM_REG_LR,base+0x20001)]:uc.reg_write(reg,val)
        uc.emu_start(symbols['ring_s1_enter']|1,base+0x20000,count=100000)
        assert uc.reg_read(UC_ARM_REG_PC)==base+0x20000
        assert uc.reg_read(UC_ARM_REG_R0)==expected,(upper_y,upper_valid)
        assert uc.mem_read(base,1)[0]==expected
    print('PASS: upper above or missing accepted; equal and below rejected despite confirmed input')

if __name__=='__main__':check()
