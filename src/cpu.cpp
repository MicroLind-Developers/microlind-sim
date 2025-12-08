#include "microlind/cpu.hpp"

#include "microlind/bus.hpp"

#include <algorithm>
#include <array>
#include <utility>

namespace microlind {

namespace {
uint8_t hi(uint16_t value) { return static_cast<uint8_t>((value >> 8) & 0xFF); }
uint8_t lo(uint16_t value) { return static_cast<uint8_t>(value & 0xFF); }
} // namespace

Cpu::Cpu(CpuMode mode) : mode_(mode) {
    // Default stack high for quick use; user can override by writing regs().s.
    regs_.s = 0xFFFF;
    regs_.cc = CC_I; // IRQ masked on reset by default.

    std::fill(std::begin(page0_), std::end(page0_), &Cpu::op_invalid);
    std::fill(std::begin(page10_), std::end(page10_), &Cpu::op_invalid);
    std::fill(std::begin(page11_), std::end(page11_), &Cpu::op_invalid);

    // Inherent
    page0_[0x12] = &Cpu::op_nop;
    page0_[0x4F] = &Cpu::op_clra;
    page0_[0x5F] = &Cpu::op_clrb;
    page0_[0x39] = &Cpu::op_rts;

    // Branches
    page0_[0x20] = &Cpu::op_bra;
    page0_[0x8D] = &Cpu::op_bsr;
    page0_[0x26] = &Cpu::op_bne;
    page0_[0x27] = &Cpu::op_beq;
    page0_[0x22] = &Cpu::op_bhi;
    page0_[0x23] = &Cpu::op_bls;
    page0_[0x24] = &Cpu::op_bcc;
    page0_[0x25] = &Cpu::op_bcs;
    page0_[0x2A] = &Cpu::op_bpl;
    page0_[0x2B] = &Cpu::op_bmi;
    page0_[0x28] = &Cpu::op_bvc;
    page0_[0x29] = &Cpu::op_bvs;
    page0_[0x2C] = &Cpu::op_bge;
    page0_[0x2D] = &Cpu::op_blt;
    page0_[0x2E] = &Cpu::op_bgt;
    page0_[0x2F] = &Cpu::op_ble;

    // Jumps
    page0_[0x0E] = &Cpu::op_jmp_dir;
    page0_[0x7E] = &Cpu::op_jmp_ext;
    page0_[0x6E] = &Cpu::op_jmp_idx;
    page0_[0x9D] = &Cpu::op_jsr_dir;
    page0_[0xBD] = &Cpu::op_jsr_ext;
    page0_[0xAD] = &Cpu::op_jsr_idx;
    page0_[0x39] = &Cpu::op_rts;
    page0_[0x3B] = &Cpu::op_rti;
    page0_[0x3F] = &Cpu::op_swi;
    page10_[0x3F] = &Cpu::op_swi2;
    page11_[0x3F] = &Cpu::op_swi3;
    page0_[0x3C] = &Cpu::op_cwai;
    page0_[0x13] = &Cpu::op_sync;
    page0_[0x3D] = &Cpu::op_mul;

    // Loads / Stores
    page0_[0x86] = &Cpu::op_lda_imm;
    page0_[0x96] = &Cpu::op_lda_dir;
    page0_[0xB6] = &Cpu::op_lda_ext;
    page0_[0xA6] = &Cpu::op_lda_idx;
    page0_[0xC6] = &Cpu::op_ldb_imm;
    page0_[0xD6] = &Cpu::op_ldb_dir;
    page0_[0xF6] = &Cpu::op_ldb_ext;
    page0_[0xE6] = &Cpu::op_ldb_idx;
    page0_[0xCC] = &Cpu::op_ldd_imm;
    page0_[0xDC] = &Cpu::op_ldd_dir;
    page0_[0xFC] = &Cpu::op_ldd_ext;
    page0_[0xEC] = &Cpu::op_ldd_idx;

    page0_[0x97] = &Cpu::op_sta_dir;
    page0_[0xB7] = &Cpu::op_sta_ext;
    page0_[0xD7] = &Cpu::op_stb_dir;
    page0_[0xF7] = &Cpu::op_stb_ext;
    page0_[0xA7] = &Cpu::op_sta_idx;
    page0_[0xE7] = &Cpu::op_stb_idx;
    page0_[0xDD] = &Cpu::op_std_dir;
    page0_[0xFD] = &Cpu::op_std_ext;
    page0_[0xED] = &Cpu::op_std_idx;

    page0_[0x1E] = &Cpu::op_exg;
    page0_[0x1F] = &Cpu::op_tfr;

    // Logical
    page0_[0x84] = &Cpu::op_anda_imm;
    page0_[0x94] = &Cpu::op_anda_dir;
    page0_[0xB4] = &Cpu::op_anda_ext;
    page0_[0xA4] = &Cpu::op_anda_idx;
    page0_[0xC4] = &Cpu::op_andb_imm;
    page0_[0xD4] = &Cpu::op_andb_dir;
    page0_[0xF4] = &Cpu::op_andb_ext;
    page0_[0xE4] = &Cpu::op_andb_idx;

    page0_[0x8A] = &Cpu::op_ora_imm;
    page0_[0x9A] = &Cpu::op_ora_dir;
    page0_[0xBA] = &Cpu::op_ora_ext;
    page0_[0xAA] = &Cpu::op_ora_idx;
    page0_[0xCA] = &Cpu::op_orb_imm;
    page0_[0xDA] = &Cpu::op_orb_dir;
    page0_[0xFA] = &Cpu::op_orb_ext;
    page0_[0xEA] = &Cpu::op_orb_idx;

    page0_[0x88] = &Cpu::op_eora_imm;
    page0_[0x98] = &Cpu::op_eora_dir;
    page0_[0xB8] = &Cpu::op_eora_ext;
    page0_[0xA8] = &Cpu::op_eora_idx;
    page0_[0xC8] = &Cpu::op_eorb_imm;
    page0_[0xD8] = &Cpu::op_eorb_dir;
    page0_[0xF8] = &Cpu::op_eorb_ext;
    page0_[0xE8] = &Cpu::op_eorb_idx;

    // Arithmetic
    page0_[0x8B] = &Cpu::op_adda_imm;
    page0_[0x9B] = &Cpu::op_adda_dir;
    page0_[0xBB] = &Cpu::op_adda_ext;
    page0_[0xAB] = &Cpu::op_adda_idx;
    page0_[0xCB] = &Cpu::op_addb_imm;
    page0_[0xDB] = &Cpu::op_addb_dir;
    page0_[0xFB] = &Cpu::op_addb_ext;
    page0_[0xEB] = &Cpu::op_addb_idx;
    page0_[0xC3] = &Cpu::op_addd_imm;
    page0_[0xD3] = &Cpu::op_addd_dir;
    page0_[0xF3] = &Cpu::op_addd_ext;
    page0_[0xE3] = &Cpu::op_addd_idx;

    page0_[0x80] = &Cpu::op_suba_imm;
    page0_[0x90] = &Cpu::op_suba_dir;
    page0_[0xB0] = &Cpu::op_suba_ext;
    page0_[0xA0] = &Cpu::op_suba_idx;
    page0_[0xC0] = &Cpu::op_subb_imm;
    page0_[0xD0] = &Cpu::op_subb_dir;
    page0_[0xF0] = &Cpu::op_subb_ext;
    page0_[0xE0] = &Cpu::op_subb_idx;
    page0_[0x83] = &Cpu::op_subd_imm;
    page0_[0x93] = &Cpu::op_subd_dir;
    page0_[0xB3] = &Cpu::op_subd_ext;
    page0_[0xA3] = &Cpu::op_subd_idx;

    page0_[0x81] = &Cpu::op_cmpa_imm;
    page0_[0x91] = &Cpu::op_cmpa_dir;
    page0_[0xB1] = &Cpu::op_cmpa_ext;
    page0_[0xA1] = &Cpu::op_cmpa_idx;
    page0_[0xC1] = &Cpu::op_cmpb_imm;
    page0_[0xD1] = &Cpu::op_cmpb_dir;
    page0_[0xF1] = &Cpu::op_cmpb_ext;
    page0_[0xE1] = &Cpu::op_cmpb_idx;
    page10_[0x83] = &Cpu::op_cmpd_imm;
    page10_[0x93] = &Cpu::op_cmpd_dir;
    page10_[0xB3] = &Cpu::op_cmpd_ext;
    page10_[0xA3] = &Cpu::op_cmpd_idx;

    // 6309 arithmetic / logic extensions
    page0_[0xCD] = &Cpu::op_ldq_imm;
    page10_[0xDC] = &Cpu::op_ldq_dir;
    page10_[0xEC] = &Cpu::op_ldq_idx;
    page10_[0xFC] = &Cpu::op_ldq_ext;
    page10_[0xDD] = &Cpu::op_stq_dir;
    page10_[0xED] = &Cpu::op_stq_idx;
    page10_[0xFD] = &Cpu::op_stq_ext;

    page10_[0x86] = &Cpu::op_ldw_imm;
    page10_[0x96] = &Cpu::op_ldw_dir;
    page10_[0xA6] = &Cpu::op_ldw_idx;
    page10_[0xB6] = &Cpu::op_ldw_ext;
    page10_[0x97] = &Cpu::op_stw_dir;
    page10_[0xA7] = &Cpu::op_stw_idx;
    page10_[0xB7] = &Cpu::op_stw_ext;

    page10_[0x8B] = &Cpu::op_addw_imm;
    page10_[0x9B] = &Cpu::op_addw_dir;
    page10_[0xAB] = &Cpu::op_addw_idx;
    page10_[0xBB] = &Cpu::op_addw_ext;
    page10_[0x80] = &Cpu::op_subw_imm;
    page10_[0x90] = &Cpu::op_subw_dir;
    page10_[0xA0] = &Cpu::op_subw_idx;
    page10_[0xB0] = &Cpu::op_subw_ext;
    page10_[0x81] = &Cpu::op_cmpw_imm;
    page10_[0x91] = &Cpu::op_cmpw_dir;
    page10_[0xA1] = &Cpu::op_cmpw_idx;
    page10_[0xB1] = &Cpu::op_cmpw_ext;

    page11_[0x8B] = &Cpu::op_adde_imm;
    page11_[0x9B] = &Cpu::op_adde_dir;
    page11_[0xAB] = &Cpu::op_adde_idx;
    page11_[0xBB] = &Cpu::op_adde_ext;
    page11_[0xCB] = &Cpu::op_addf_imm;
    page11_[0xDB] = &Cpu::op_addf_dir;
    page11_[0xEB] = &Cpu::op_addf_idx;
    page11_[0xFB] = &Cpu::op_addf_ext;

    page11_[0x80] = &Cpu::op_sube_imm;
    page11_[0x90] = &Cpu::op_sube_dir;
    page11_[0xA0] = &Cpu::op_sube_idx;
    page11_[0xB0] = &Cpu::op_sube_ext;
    page11_[0xC0] = &Cpu::op_subf_imm;
    page11_[0xD0] = &Cpu::op_subf_dir;
    page11_[0xE0] = &Cpu::op_subf_idx;
    page11_[0xF0] = &Cpu::op_subf_ext;

    page11_[0x81] = &Cpu::op_cmpe_imm;
    page11_[0x91] = &Cpu::op_cmpe_dir;
    page11_[0xA1] = &Cpu::op_cmpe_idx;
    page11_[0xB1] = &Cpu::op_cmpe_ext;
    page11_[0xC1] = &Cpu::op_cmpf_imm;
    page11_[0xD1] = &Cpu::op_cmpf_dir;
    page11_[0xE1] = &Cpu::op_cmpf_idx;
    page11_[0xF1] = &Cpu::op_cmpf_ext;

    page10_[0x89] = &Cpu::op_adcd_imm;
    page10_[0x99] = &Cpu::op_adcd_dir;
    page10_[0xA9] = &Cpu::op_adcd_idx;
    page10_[0xB9] = &Cpu::op_adcd_ext;
    page10_[0x82] = &Cpu::op_sbcd_imm;
    page10_[0x92] = &Cpu::op_sbcd_dir;
    page10_[0xA2] = &Cpu::op_sbcd_idx;
    page10_[0xB2] = &Cpu::op_sbcd_ext;
    page10_[0x8A] = &Cpu::op_ord_imm;
    page10_[0x9A] = &Cpu::op_ord_dir;
    page10_[0xAA] = &Cpu::op_ord_idx;
    page10_[0xBA] = &Cpu::op_ord_ext;

    page11_[0x8E] = &Cpu::op_divq_imm;
    page11_[0x9E] = &Cpu::op_divq_dir;
    page11_[0xAE] = &Cpu::op_divq_idx;
    page11_[0xBE] = &Cpu::op_divq_ext;

    page11_[0x8F] = &Cpu::op_muld_imm;
    page11_[0x9F] = &Cpu::op_muld_dir;
    page11_[0xAF] = &Cpu::op_muld_idx;
    page11_[0xBF] = &Cpu::op_muld_ext;

    page10_[0x30] = &Cpu::op_addr;
    page10_[0x32] = &Cpu::op_subr;
    page10_[0x37] = &Cpu::op_cmpr;
    page10_[0x33] = &Cpu::op_sbcr;
    page10_[0x31] = &Cpu::op_adcr;
    page10_[0x35] = &Cpu::op_orr;

    // Shift/rotate 6309
    page10_[0x48] = &Cpu::op_lsl_d;
    page10_[0x49] = &Cpu::op_rold;
    page10_[0x46] = &Cpu::op_rord;
    page10_[0x58] = &Cpu::op_lslw_inh;
    page10_[0x59] = &Cpu::op_rolw;
    page10_[0x56] = &Cpu::op_rorw;
    page10_[0x5C] = &Cpu::op_incw_inh;
    page10_[0x5D] = &Cpu::op_tstw_inh;
    page10_[0x5F] = &Cpu::op_clrw_inh;

    // LDMD / SEXW
    page11_[0x3D] = &Cpu::op_ldmd;
    page0_[0x14] = &Cpu::op_sexw;

    // TFM
    page11_[0x38] = &Cpu::op_tfm_pp;
    page11_[0x39] = &Cpu::op_tfm_mm;
    page11_[0x3A] = &Cpu::op_tfm_pn;
    page11_[0x3B] = &Cpu::op_tfm_np;

    // Bit ops
    page0_[0x01] = &Cpu::op_oim_dir;
    page0_[0x61] = &Cpu::op_oim_idx;
    page0_[0x71] = &Cpu::op_oim_ext;
    page0_[0x02] = &Cpu::op_aim_dir;
    page0_[0x62] = &Cpu::op_aim_idx;
    page0_[0x72] = &Cpu::op_aim_ext;
    page0_[0x05] = &Cpu::op_eim_dir;
    page0_[0x65] = &Cpu::op_eim_idx;
    page0_[0x75] = &Cpu::op_eim_ext;
    page0_[0x0B] = &Cpu::op_tim_dir;
    page0_[0x6B] = &Cpu::op_tim_idx;
    page0_[0x7B] = &Cpu::op_tim_ext;

    // Bit transfer/logic (direct only) 0x11 prefix
    page11_[0x30] = &Cpu::op_band;
    page11_[0x31] = &Cpu::op_biand;
    page11_[0x32] = &Cpu::op_bor;
    page11_[0x33] = &Cpu::op_bior;
    page11_[0x34] = &Cpu::op_beor;
    page11_[0x35] = &Cpu::op_bieor;
    page11_[0x36] = &Cpu::op_ldbt;
    page11_[0x37] = &Cpu::op_stbt;

    page0_[0x89] = &Cpu::op_adca_imm;
    page0_[0x99] = &Cpu::op_adca_dir;
    page0_[0xB9] = &Cpu::op_adca_ext;
    page0_[0xA9] = &Cpu::op_adca_idx;
    page0_[0xC9] = &Cpu::op_adcb_imm;
    page0_[0xD9] = &Cpu::op_adcb_dir;
    page0_[0xF9] = &Cpu::op_adcb_ext;
    page0_[0xE9] = &Cpu::op_adcb_idx;

    page0_[0x82] = &Cpu::op_sbca_imm;
    page0_[0x92] = &Cpu::op_sbca_dir;
    page0_[0xB2] = &Cpu::op_sbca_ext;
    page0_[0xA2] = &Cpu::op_sbca_idx;
    page0_[0xC2] = &Cpu::op_sbcb_imm;
    page0_[0xD2] = &Cpu::op_sbcb_dir;
    page0_[0xF2] = &Cpu::op_sbcb_ext;
    page0_[0xE2] = &Cpu::op_sbcb_idx;

    page0_[0x85] = &Cpu::op_bita_imm;
    page0_[0x95] = &Cpu::op_bita_dir;
    page0_[0xB5] = &Cpu::op_bita_ext;
    page0_[0xA5] = &Cpu::op_bita_idx;
    page0_[0xC5] = &Cpu::op_bitb_imm;
    page0_[0xD5] = &Cpu::op_bitb_dir;
    page0_[0xF5] = &Cpu::op_bitb_ext;
    page0_[0xE5] = &Cpu::op_bitb_idx;

    // LEA
    page0_[0x30] = &Cpu::op_leax;
    page0_[0x31] = &Cpu::op_leay;
    page0_[0x32] = &Cpu::op_leas;
    page0_[0x33] = &Cpu::op_leau;

    // 16-bit loads/stores
    page0_[0x8E] = &Cpu::op_ldx_imm;
    page0_[0x9E] = &Cpu::op_ldx_dir;
    page0_[0xBE] = &Cpu::op_ldx_ext;
    page0_[0xAE] = &Cpu::op_ldx_idx;
    page0_[0xCE] = &Cpu::op_ldu_imm;
    page0_[0xDE] = &Cpu::op_ldu_dir;
    page0_[0xFE] = &Cpu::op_ldu_ext;
    page0_[0xEE] = &Cpu::op_ldu_idx;
    page10_[0x8E] = &Cpu::op_ldy_imm;
    page10_[0x9E] = &Cpu::op_ldy_dir;
    page10_[0xBE] = &Cpu::op_ldy_ext;
    page10_[0xAE] = &Cpu::op_ldy_idx;
    page10_[0xCE] = &Cpu::op_lds_imm;
    page10_[0xDE] = &Cpu::op_lds_dir;
    page10_[0xFE] = &Cpu::op_lds_ext;
    page10_[0xEE] = &Cpu::op_lds_idx;

    page0_[0x9F] = &Cpu::op_stx_dir;
    page0_[0xBF] = &Cpu::op_stx_ext;
    page0_[0xAF] = &Cpu::op_stx_idx;
    page0_[0xDF] = &Cpu::op_stu_dir;
    page0_[0xFF] = &Cpu::op_stu_ext;
    page0_[0xEF] = &Cpu::op_stu_idx;
    page10_[0x9F] = &Cpu::op_sty_dir;
    page10_[0xBF] = &Cpu::op_sty_ext;
    page10_[0xAF] = &Cpu::op_sty_idx;
    page10_[0xDF] = &Cpu::op_sts_dir;
    page10_[0xFF] = &Cpu::op_sts_ext;
    page10_[0xEF] = &Cpu::op_sts_idx;

    // Compare 16-bit
    page0_[0x8C] = &Cpu::op_cmpx_imm;
    page0_[0x9C] = &Cpu::op_cmpx_dir;
    page0_[0xBC] = &Cpu::op_cmpx_ext;
    page0_[0xAC] = &Cpu::op_cmpx_idx;
    page10_[0x8C] = &Cpu::op_cmpy_imm;
    page10_[0x9C] = &Cpu::op_cmpy_dir;
    page10_[0xBC] = &Cpu::op_cmpy_ext;
    page10_[0xAC] = &Cpu::op_cmpy_idx;
    page11_[0x8C] = &Cpu::op_cmpu_imm;
    page11_[0x9C] = &Cpu::op_cmpu_dir;
    page11_[0xBC] = &Cpu::op_cmpu_ext;
    page11_[0xAC] = &Cpu::op_cmpu_idx;
    page11_[0x8E] = &Cpu::op_cmps_imm;
    page11_[0x9E] = &Cpu::op_cmps_dir;
    page11_[0xBE] = &Cpu::op_cmps_ext;
    page11_[0xAE] = &Cpu::op_cmps_idx;

    // Misc
    page0_[0x3A] = &Cpu::op_abx;
    page0_[0x1D] = &Cpu::op_sex;
    page0_[0x1C] = &Cpu::op_andcc;
    page0_[0x1A] = &Cpu::op_orcc;
    page0_[0x19] = &Cpu::op_daa;

    // Stack
    page0_[0x34] = &Cpu::op_pshs;
    page0_[0x35] = &Cpu::op_puls;
    page0_[0x36] = &Cpu::op_pshu;
    page0_[0x37] = &Cpu::op_pulu;

    // Accumulator unary/shift
    page0_[0x40] = &Cpu::op_nega;
    page0_[0x50] = &Cpu::op_negb;
    page0_[0x43] = &Cpu::op_coma;
    page0_[0x53] = &Cpu::op_comb;
    page0_[0x44] = &Cpu::op_lsra;
    page0_[0x54] = &Cpu::op_lsrb;
    page0_[0x46] = &Cpu::op_rora;
    page0_[0x56] = &Cpu::op_rorb;
    page0_[0x47] = &Cpu::op_asra;
    page0_[0x57] = &Cpu::op_asrb;
    page0_[0x48] = &Cpu::op_asla;
    page0_[0x58] = &Cpu::op_aslb;
    page0_[0x49] = &Cpu::op_rola;
    page0_[0x59] = &Cpu::op_rolb;
    page0_[0x4A] = &Cpu::op_deca;
    page0_[0x5A] = &Cpu::op_decb;
    page0_[0x4C] = &Cpu::op_inca;
    page0_[0x5C] = &Cpu::op_incb;
    page0_[0x4D] = &Cpu::op_tsta;
    page0_[0x5D] = &Cpu::op_tstb;

    // Memory unary/shift
    page0_[0x00] = &Cpu::op_neg_dir;
    page0_[0x60] = &Cpu::op_neg_idx;
    page0_[0x70] = &Cpu::op_neg_ext;
    page0_[0x03] = &Cpu::op_com_dir;
    page0_[0x63] = &Cpu::op_com_idx;
    page0_[0x73] = &Cpu::op_com_ext;
    page0_[0x04] = &Cpu::op_lsr_dir;
    page0_[0x64] = &Cpu::op_lsr_idx;
    page0_[0x74] = &Cpu::op_lsr_ext;
    page0_[0x06] = &Cpu::op_ror_dir;
    page0_[0x66] = &Cpu::op_ror_idx;
    page0_[0x76] = &Cpu::op_ror_ext;
    page0_[0x07] = &Cpu::op_asr_dir;
    page0_[0x67] = &Cpu::op_asr_idx;
    page0_[0x77] = &Cpu::op_asr_ext;
    page0_[0x08] = &Cpu::op_asl_dir;
    page0_[0x68] = &Cpu::op_asl_idx;
    page0_[0x78] = &Cpu::op_asl_ext;
    page0_[0x09] = &Cpu::op_rol_dir;
    page0_[0x69] = &Cpu::op_rol_idx;
    page0_[0x79] = &Cpu::op_rol_ext;
    page0_[0x0A] = &Cpu::op_dec_dir;
    page0_[0x6A] = &Cpu::op_dec_idx;
    page0_[0x7A] = &Cpu::op_dec_ext;
    page0_[0x0C] = &Cpu::op_inc_dir;
    page0_[0x6C] = &Cpu::op_inc_idx;
    page0_[0x7C] = &Cpu::op_inc_ext;
    page0_[0x0D] = &Cpu::op_tst_dir;
    page0_[0x6D] = &Cpu::op_tst_idx;
    page0_[0x7D] = &Cpu::op_tst_ext;
    page0_[0x0F] = &Cpu::op_clr_dir;
    page0_[0x6F] = &Cpu::op_clr_idx;
    page0_[0x7F] = &Cpu::op_clr_ext;
}

CpuTickResult Cpu::tick(Bus& bus) {
    last_pc_ = regs_.pc;
    const uint8_t opcode = fetch_byte(bus);
    last_prefix_ = 0x00;
    last_opcode_ = opcode;
    Handler handler = &Cpu::op_invalid;
    if (opcode == 0x10) {
        const uint8_t next = fetch_byte(bus);
        handler = page10_[next];
        last_prefix_ = 0x10;
        last_opcode_ = next;
    } else if (opcode == 0x11) {
        const uint8_t next = fetch_byte(bus);
        handler = page11_[next];
        last_prefix_ = 0x11;
        last_opcode_ = next;
    } else {
        handler = page0_[opcode];
    }

    const uint8_t cycles = (this->*handler)(bus);
    cycles_executed_ += cycles;
    return CpuTickResult{cycles};
}

uint8_t Cpu::fetch_byte(Bus& bus) {
    const uint8_t value = read_byte(bus, regs_.pc);
    regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
    return value;
}

uint16_t Cpu::fetch_word(Bus& bus) {
    const uint16_t high = fetch_byte(bus);
    const uint16_t low = fetch_byte(bus);
    return static_cast<uint16_t>((high << 8) | low);
}

uint8_t Cpu::read_byte(Bus& bus, uint16_t address) { return bus.read8(address); }

void Cpu::write_byte(Bus& bus, uint16_t address, uint8_t value) { bus.write8(address, value); }

static inline uint16_t read_word(Bus& bus, uint16_t address) {
    return static_cast<uint16_t>((bus.read8(address) << 8) | bus.read8(static_cast<uint16_t>(address + 1)));
}

static inline void write_word(Bus& bus, uint16_t address, uint16_t value) {
    bus.write8(address, static_cast<uint8_t>((value >> 8) & 0xFF));
    bus.write8(static_cast<uint16_t>(address + 1), static_cast<uint8_t>(value & 0xFF));
}

static constexpr uint16_t VECTOR_SWI = 0xFFFA;
static constexpr uint16_t VECTOR_SWI2 = 0xFFF4;
static constexpr uint16_t VECTOR_SWI3 = 0xFFF2;

uint16_t Cpu::direct_address(Bus& bus) {
    const uint8_t offset = fetch_byte(bus);
    return static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | offset);
}

uint16_t Cpu::extended_address(Bus& bus) { return fetch_word(bus); }

Cpu::PostbyteResult Cpu::indexed_address(Bus& bus) {
    const uint8_t post = fetch_byte(bus);

    // 5-bit offset mode when bit7 == 0.
    if ((post & 0x80) == 0) {
        const IndexReg base = static_cast<IndexReg>((post >> 5) & 0x03);
        const int8_t offset = static_cast<int8_t>((post << 3)) >> 3; // sign-extend 5 bits
        const uint16_t addr = static_cast<uint16_t>(index_value(base) + offset);
        return {addr, 1};
    }

    IndexReg base = IndexReg::X;
    switch ((post >> 5) & 0x03) {
    case 0: base = IndexReg::X; break;
    case 1: base = IndexReg::Y; break;
    case 2: base = IndexReg::U; break;
    case 3: base = IndexReg::S; break;
    }

    const bool indirect = (post & 0x10) != 0;
    uint16_t addr = 0;
    uint8_t cycles = 0;

    switch (post & 0x0F) {
    case 0x00: // ,R+
        addr = index_value(base);
        index_ref(base) = static_cast<uint16_t>(index_ref(base) + 1);
        cycles = 2;
        break;
    case 0x01: // ,R++
        addr = index_value(base);
        index_ref(base) = static_cast<uint16_t>(index_ref(base) + 2);
        cycles = 3;
        break;
    case 0x02: // ,-R
        index_ref(base) = static_cast<uint16_t>(index_ref(base) - 1);
        addr = index_value(base);
        cycles = 2;
        break;
    case 0x03: // ,--R
        index_ref(base) = static_cast<uint16_t>(index_ref(base) - 2);
        addr = index_value(base);
        cycles = 3;
        break;
    case 0x04: // ,R
        addr = index_value(base);
        cycles = 0;
        break;
    case 0x05: // B,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.b));
        cycles = 1;
        break;
    case 0x06: // A,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(regs_.a));
        cycles = 1;
        break;
    case 0x08: // n,R 8-bit offset
        addr = static_cast<uint16_t>(index_value(base) + static_cast<int8_t>(fetch_byte(bus)));
        cycles = 1;
        break;
    case 0x09: // n,R 16-bit offset
        addr = static_cast<uint16_t>(index_value(base) + fetch_word(bus));
        cycles = 4;
        break;
    case 0x0B: // D,R
        addr = static_cast<uint16_t>(index_value(base) + static_cast<uint16_t>((regs_.a << 8) | regs_.b));
        cycles = 4;
        break;
    case 0x0C: // ,PC 8-bit
        addr = static_cast<uint16_t>(regs_.pc + static_cast<int8_t>(fetch_byte(bus)));
        cycles = 1;
        base = IndexReg::PC;
        break;
    case 0x0D: // ,PC 16-bit
        addr = static_cast<uint16_t>(regs_.pc + fetch_word(bus));
        cycles = 4;
        base = IndexReg::PC;
        break;
    case 0x0F: // Extended
        addr = fetch_word(bus);
        cycles = 5;
        base = IndexReg::PC;
        break;
    default:
        // Many modes not yet handled (e.g., [n,PC], [n,R], W-reg modes on 6309).
        addr = index_value(base);
        cycles = 0;
        break;
    }

    if (indirect) {
        const uint16_t indirect_addr = addr;
        addr = static_cast<uint16_t>((read_byte(bus, indirect_addr) << 8) | read_byte(bus, static_cast<uint16_t>(indirect_addr + 1)));
        cycles = static_cast<uint8_t>(cycles + 3);
    }

    return {addr, cycles};
}

void Cpu::push_byte(Bus& bus, uint8_t value) {
    regs_.s = static_cast<uint16_t>(regs_.s - 1);
    write_byte(bus, regs_.s, value);
}

void Cpu::push_word(Bus& bus, uint16_t value) {
    push_byte(bus, lo(value));
    push_byte(bus, hi(value));
}

uint8_t Cpu::pull_byte(Bus& bus) {
    const uint8_t value = read_byte(bus, regs_.s);
    regs_.s = static_cast<uint16_t>(regs_.s + 1);
    return value;
}

uint16_t Cpu::pull_word(Bus& bus) {
    const uint8_t high = pull_byte(bus);
    const uint8_t low = pull_byte(bus);
    return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void Cpu::set_flags_nz8(uint8_t value) {
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (value == 0) {
        regs_.cc |= CC_Z;
    }
    if (value & 0x80) {
        regs_.cc |= CC_N;
    }
}

void Cpu::set_flags_nz16(uint16_t value) {
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (value == 0) {
        regs_.cc |= CC_Z;
    }
    if (value & 0x8000) {
        regs_.cc |= CC_N;
    }
}

uint8_t Cpu::branch_if(Bus& bus, bool take) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    if (take) {
        regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
        return 3;
    }
    return 2;
}

static inline void flags_logic8(Registers& r, uint8_t value) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (value == 0) r.cc |= CC_Z;
    if (value & 0x80) r.cc |= CC_N;
}

static inline uint8_t add8(Registers& r, uint8_t a, uint8_t b) {
    const uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b);
    const uint8_t res = static_cast<uint8_t>(sum & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C | CC_H));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x80) && !((a ^ b) & 0x80)) r.cc |= CC_V;
    if (sum & 0x100) r.cc |= CC_C;
    if (((a & b) | (b & ~res) | (a & ~res)) & 0x10) r.cc |= CC_H;
    return res;
}

static inline uint8_t sub8(Registers& r, uint8_t a, uint8_t b) {
    const uint16_t diff = static_cast<uint16_t>(a) - static_cast<uint16_t>(b);
    const uint8_t res = static_cast<uint8_t>(diff & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x80) != 0) r.cc |= CC_V;
    if (diff & 0x100) r.cc |= CC_C;
    return res;
}

static inline uint16_t add16(Registers& r, uint16_t a, uint16_t b) {
    const uint32_t sum = static_cast<uint32_t>(a) + static_cast<uint32_t>(b);
    const uint16_t res = static_cast<uint16_t>(sum & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x8000) && !((a ^ b) & 0x8000)) r.cc |= CC_V;
    if (sum & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t sub16(Registers& r, uint16_t a, uint16_t b) {
    const uint32_t diff = static_cast<uint32_t>(a) - static_cast<uint32_t>(b);
    const uint16_t res = static_cast<uint16_t>(diff & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x8000) != 0) r.cc |= CC_V;
    if (diff & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t add16_carry(Registers& r, uint16_t a, uint16_t b, bool carry_in) {
    const uint32_t sum = static_cast<uint32_t>(a) + static_cast<uint32_t>(b) + (carry_in ? 1u : 0u);
    const uint16_t res = static_cast<uint16_t>(sum & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x8000) && !((a ^ b) & 0x8000)) r.cc |= CC_V;
    if (sum & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint16_t sub16_carry(Registers& r, uint16_t a, uint16_t b, bool carry_in) {
    const uint32_t diff = static_cast<uint32_t>(a) - static_cast<uint32_t>(b) - (carry_in ? 1u : 0u);
    const uint16_t res = static_cast<uint16_t>(diff & 0xFFFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x8000) != 0) r.cc |= CC_V;
    if (diff & 0x10000) r.cc |= CC_C;
    return res;
}

static inline uint8_t adc8(Registers& r, uint8_t a, uint8_t b) {
    const uint8_t carry = (r.cc & CC_C) ? 1 : 0;
    const uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b) + carry;
    const uint8_t res = static_cast<uint8_t>(sum & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C | CC_H));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b ^ res) & 0x80) && !((a ^ b) & 0x80)) r.cc |= CC_V;
    if (sum & 0x100) r.cc |= CC_C;
    if (((a & b) | (b & ~res) | (a & ~res)) & 0x10) r.cc |= CC_H;
    return res;
}

static inline uint8_t sbc8(Registers& r, uint8_t a, uint8_t b) {
    const uint8_t carry = (r.cc & CC_C) ? 1 : 0;
    const uint16_t diff = static_cast<uint16_t>(a) - static_cast<uint16_t>(b) - carry;
    const uint8_t res = static_cast<uint8_t>(diff & 0xFF);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (((a ^ b) & (a ^ res) & 0x80) != 0) r.cc |= CC_V;
    if (diff & 0x100) r.cc |= CC_C;
    return res;
}

static inline uint8_t dec8(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(v - 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0x7F) r.cc |= CC_V;
    return res;
}

static inline uint8_t inc8(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(v + 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) r.cc |= CC_Z;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0x80) r.cc |= CC_V;
    return res;
}

static inline void set_flags_tst(Registers& r, uint8_t v) {
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (v == 0) r.cc |= CC_Z;
    if (v & 0x80) r.cc |= CC_N;
}

static inline uint8_t neg8_op(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(0u - v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (v == 0x80) r.cc |= CC_V;
    if (v != 0) r.cc |= CC_C;
    return res;
}

static inline uint8_t com8_op(Registers& r, uint8_t v) {
    const uint8_t res = static_cast<uint8_t>(~v);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    r.cc |= CC_C;
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint8_t lsr8_op(Registers& r, uint8_t v) {
    const uint8_t c = v & 0x01;
    const uint8_t res = static_cast<uint8_t>(v >> 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    r.cc |= c;
    if (res == 0) r.cc |= CC_Z;
    return res;
}

static inline uint8_t ror8_op(Registers& r, uint8_t v) {
    const uint8_t c_in = (r.cc & CC_C) ? 0x80 : 0x00;
    const uint8_t c_out = v & 0x01;
    const uint8_t res = static_cast<uint8_t>((v >> 1) | c_in);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}

static inline uint8_t asr8_op(Registers& r, uint8_t v) {
    const uint8_t c_out = v & 0x01;
    const uint8_t res = static_cast<uint8_t>((v >> 1) | (v & 0x80));
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    return res;
}

static inline uint8_t asl8_op(Registers& r, uint8_t v) {
    const uint8_t c_out = static_cast<uint8_t>((v >> 7) & 0x01);
    const uint8_t res = static_cast<uint8_t>(v << 1);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}

static inline uint8_t rol8_op(Registers& r, uint8_t v) {
    const uint8_t c_in = (r.cc & CC_C) ? 1 : 0;
    const uint8_t c_out = static_cast<uint8_t>((v >> 7) & 0x01);
    const uint8_t res = static_cast<uint8_t>((v << 1) | c_in);
    r.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x80) r.cc |= CC_N;
    if (res == 0) r.cc |= CC_Z;
    if (c_out) r.cc |= CC_C;
    if (((res ^ (c_out ? 0x80 : 0x00)) & 0x80) != 0) r.cc |= CC_V; // V = N xor C
    return res;
}

uint16_t& Cpu::index_ref(IndexReg reg) {
    switch (reg) {
    case IndexReg::X: return regs_.x;
    case IndexReg::Y: return regs_.y;
    case IndexReg::U: return regs_.u;
    case IndexReg::S: return regs_.s;
    case IndexReg::PC: return regs_.pc;
    }
    return regs_.x;
}

uint16_t Cpu::index_value(IndexReg reg) const {
    switch (reg) {
    case IndexReg::X: return regs_.x;
    case IndexReg::Y: return regs_.y;
    case IndexReg::U: return regs_.u;
    case IndexReg::S: return regs_.s;
    case IndexReg::PC: return regs_.pc;
    }
    return regs_.x;
}

uint8_t* Cpu::reg8_by_code(uint8_t code) {
    // 0:D,1:X,2:Y,3:U,4:S,5:PC,8:A,9:B,A:CC,B:DP (per 6809 rules)
    switch (code & 0x0F) {
    case 0x08: return &regs_.a;
    case 0x09: return &regs_.b;
    case 0x0A: return &regs_.cc;
    case 0x0B: return &regs_.dp;
    case 0x0C: return &regs_.e; // 6309
    case 0x0D: return &regs_.f; // 6309
    default: return nullptr;
    }
}

bool Cpu::get_reg16_by_code(uint8_t code, uint16_t& out) const {
    switch (code & 0x0F) {
    case 0x00: out = static_cast<uint16_t>((regs_.a << 8) | regs_.b); return true; // D
    case 0x01: out = regs_.x; return true;
    case 0x02: out = regs_.y; return true;
    case 0x03: out = regs_.u; return true;
    case 0x04: out = regs_.s; return true;
    case 0x05: out = regs_.pc; return true;
    case 0x06: out = reg_w(); return true; // W (E:F)
    case 0x07: out = 0; return true;       // V treated as zero register
    default: return false;
    }
}

bool Cpu::set_reg16_by_code(uint8_t code, uint16_t value) {
    switch (code & 0x0F) {
    case 0x00: regs_.a = hi(value); regs_.b = lo(value); return true;
    case 0x01: regs_.x = value; return true;
    case 0x02: regs_.y = value; return true;
    case 0x03: regs_.u = value; return true;
    case 0x04: regs_.s = value; return true;
    case 0x05: regs_.pc = value; return true;
    case 0x06: set_reg_w(value); return true;
    case 0x07: return true; // V ignored
    default: return false;
    }
}

bool Cpu::set_reg32_by_code(uint8_t code, uint32_t value) {
    if ((code & 0x0F) == 0x06) { // Q treated via W code
        set_reg_q(value);
        return true;
    }
    return false;
}

uint16_t Cpu::reg_w() const {
    return static_cast<uint16_t>((static_cast<uint16_t>(regs_.e) << 8) | regs_.f);
}

void Cpu::set_reg_w(uint16_t value) {
    regs_.e = hi(value);
    regs_.f = lo(value);
}

uint32_t Cpu::reg_q() const {
    const uint32_t high = static_cast<uint32_t>((regs_.a << 8) | regs_.b);
    const uint32_t low = static_cast<uint32_t>((regs_.e << 8) | regs_.f);
    return (high << 16) | low;
}

void Cpu::set_reg_q(uint32_t value) {
    regs_.a = static_cast<uint8_t>((value >> 24) & 0xFF);
    regs_.b = static_cast<uint8_t>((value >> 16) & 0xFF);
    regs_.e = static_cast<uint8_t>((value >> 8) & 0xFF);
    regs_.f = static_cast<uint8_t>(value & 0xFF);
}

bool reg_is_16bit(uint8_t code) {
    const uint8_t c = code & 0x0F;
    return c <= 0x07; // D,X,Y,U,S,PC,W,V(ignored)
}

uint16_t read_reg_for_dest(const Registers& regs, uint8_t src_code, bool dest_is_16) {
    const uint8_t c = src_code & 0x0F;
    if (dest_is_16) {
        switch (c) {
        case 0x00: return static_cast<uint16_t>((regs.a << 8) | regs.b); // D
        case 0x01: return regs.x;
        case 0x02: return regs.y;
        case 0x03: return regs.u;
        case 0x04: return regs.s;
        case 0x05: return regs.pc;
        case 0x06: return static_cast<uint16_t>((regs.e << 8) | regs.f); // W
        case 0x07: return 0;                                            // V
        case 0x08: return static_cast<uint16_t>((regs.a << 8) | regs.b); // A -> D
        case 0x09: return static_cast<uint16_t>((regs.a << 8) | regs.b); // B -> D
        case 0x0A: return regs.cc;
        case 0x0B: return static_cast<uint16_t>(regs.dp) << 8;
        case 0x0C: return static_cast<uint16_t>((regs.e << 8) | regs.f); // E -> W
        case 0x0D: return static_cast<uint16_t>((regs.e << 8) | regs.f); // F -> W
        default: return 0;
        }
    }
    // dest is 8-bit; return lower byte of source
    switch (c) {
    case 0x00: return regs.b; // D low
    case 0x01: return regs.x & 0xFF;
    case 0x02: return regs.y & 0xFF;
    case 0x03: return regs.u & 0xFF;
    case 0x04: return regs.s & 0xFF;
    case 0x05: return regs.pc & 0xFF;
    case 0x06: return regs.f; // W low
    case 0x07: return 0;
    case 0x08: return regs.a;
    case 0x09: return regs.b;
    case 0x0A: return regs.cc;
    case 0x0B: return regs.dp;
    case 0x0C: return regs.e;
    case 0x0D: return regs.f;
    default: return 0;
    }
}

void write_reg_sized(Cpu& cpu, uint8_t dest_code, uint16_t value, bool dest_is_16) {
    const uint8_t c = dest_code & 0x0F;
    if (dest_is_16) {
        cpu.set_reg16_by_code(c, value);
    } else {
        if (auto* r = cpu.reg8_by_code(c)) {
            *r = static_cast<uint8_t>(value & 0xFF);
        }
    }
}

// ---------- Instruction implementations ----------

uint8_t Cpu::op_invalid(Bus&) { return 1; }

uint8_t Cpu::op_nop(Bus&) { return 2; }

uint8_t Cpu::op_clra(Bus&) {
    regs_.a = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
    regs_.cc |= CC_Z;
    return 2;
}

uint8_t Cpu::op_clrb(Bus&) {
    regs_.b = 0;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
    regs_.cc |= CC_Z;
    return 2;
}

uint8_t Cpu::op_lda_imm(Bus& bus) {
    regs_.a = fetch_byte(bus);
    set_flags_nz8(regs_.a);
    return 2;
}

uint8_t Cpu::op_lda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = read_byte(bus, addr);
    set_flags_nz8(regs_.a);
    return 4;
}

uint8_t Cpu::op_lda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = read_byte(bus, addr);
    set_flags_nz8(regs_.a);
    return 5;
}

uint8_t Cpu::op_lda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a = read_byte(bus, pb.address);
    set_flags_nz8(regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_ldb_imm(Bus& bus) {
    regs_.b = fetch_byte(bus);
    set_flags_nz8(regs_.b);
    return 2;
}

uint8_t Cpu::op_ldb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = read_byte(bus, addr);
    set_flags_nz8(regs_.b);
    return 4;
}

uint8_t Cpu::op_ldb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = read_byte(bus, addr);
    set_flags_nz8(regs_.b);
    return 5;
}

uint8_t Cpu::op_ldb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b = read_byte(bus, pb.address);
    set_flags_nz8(regs_.b);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_ldd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 3;
}

uint8_t Cpu::op_ldd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 5;
}

uint8_t Cpu::op_ldd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return 6;
}

uint8_t Cpu::op_ldd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    regs_.a = hi(value);
    regs_.b = lo(value);
    set_flags_nz16(value);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sta_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.a);
    set_flags_nz8(regs_.a);
    return 4;
}

uint8_t Cpu::op_sta_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.a);
    set_flags_nz8(regs_.a);
    return 5;
}

uint8_t Cpu::op_sta_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_byte(bus, pb.address, regs_.a);
    set_flags_nz8(regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_stb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.b);
    set_flags_nz8(regs_.b);
    return 4;
}

uint8_t Cpu::op_stb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.b);
    set_flags_nz8(regs_.b);
    return 5;
}

uint8_t Cpu::op_stb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_byte(bus, pb.address, regs_.b);
    set_flags_nz8(regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_std_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, regs_.a);
    write_byte(bus, static_cast<uint16_t>(addr + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 5;
}

uint8_t Cpu::op_std_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, regs_.a);
    write_byte(bus, static_cast<uint16_t>(addr + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 6;
}

uint8_t Cpu::op_std_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_byte(bus, pb.address, regs_.a);
    write_byte(bus, static_cast<uint16_t>(pb.address + 1), regs_.b);
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_jmp_dir(Bus& bus) {
    regs_.pc = direct_address(bus);
    return 3;
}

uint8_t Cpu::op_jmp_ext(Bus& bus) {
    regs_.pc = extended_address(bus);
    return 4;
}

uint8_t Cpu::op_jmp_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.pc = pb.address;
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_jsr_dir(Bus& bus) {
    const uint16_t target = direct_address(bus);
    push_word(bus, regs_.pc);
    regs_.pc = target;
    return 5;
}

uint8_t Cpu::op_jsr_ext(Bus& bus) {
    const uint16_t target = extended_address(bus);
    push_word(bus, regs_.pc);
    regs_.pc = target;
    return 7;
}

uint8_t Cpu::op_jsr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    push_word(bus, regs_.pc);
    regs_.pc = pb.address;
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_rts(Bus& bus) {
    regs_.pc = pull_word(bus);
    return 5;
}

uint8_t Cpu::op_rti(Bus& bus) {
    regs_.cc = pull_byte(bus);
    uint8_t cycles = 6;
    if (regs_.cc & CC_E) {
        regs_.a = pull_byte(bus);
        regs_.b = pull_byte(bus);
        regs_.dp = pull_byte(bus);
        regs_.x = pull_word(bus);
        regs_.y = pull_word(bus);
        regs_.u = pull_word(bus);
        cycles = 15;
    }
    regs_.pc = pull_word(bus);
    return cycles;
}

uint8_t Cpu::op_swi(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= static_cast<uint8_t>(CC_I | CC_F);
    regs_.pc = read_word(bus, VECTOR_SWI);
    return 19;
}

uint8_t Cpu::op_swi2(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    regs_.pc = read_word(bus, VECTOR_SWI2);
    return 20;
}

uint8_t Cpu::op_swi3(Bus& bus) {
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    regs_.pc = read_word(bus, VECTOR_SWI3);
    return 20;
}

uint8_t Cpu::op_cwai(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc &= mask;
    regs_.cc |= CC_E;
    push_word(bus, regs_.pc);
    push_word(bus, regs_.u);
    push_word(bus, regs_.y);
    push_word(bus, regs_.x);
    push_byte(bus, regs_.dp);
    push_byte(bus, regs_.b);
    push_byte(bus, regs_.a);
    push_byte(bus, regs_.cc);
    regs_.cc |= CC_I;
    // No waiting state modeled yet.
    return 19;
}

uint8_t Cpu::op_sync(Bus&) {
    sync_wait_ = true; // not modeled yet
    return 2;
}

uint8_t Cpu::op_mul(Bus&) {
    const uint16_t res = static_cast<uint16_t>(regs_.a) * static_cast<uint16_t>(regs_.b);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_Z | CC_C | CC_V | CC_N));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res & 0x80) regs_.cc |= CC_C;
    return 11;
}

uint8_t Cpu::op_bra(Bus& bus) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
    return 3; // unconditional relative
}

uint8_t Cpu::op_bsr(Bus& bus) {
    const int8_t offset = static_cast<int8_t>(fetch_byte(bus));
    push_word(bus, regs_.pc);
    regs_.pc = static_cast<uint16_t>(regs_.pc + offset);
    return 7;
}

uint8_t Cpu::op_bne(Bus& bus) {
    return branch_if(bus, (regs_.cc & CC_Z) == 0);
}

uint8_t Cpu::op_beq(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) != 0); }
uint8_t Cpu::op_bhi(Bus& bus) { return branch_if(bus, (regs_.cc & (CC_Z | CC_C)) == 0); }
uint8_t Cpu::op_bls(Bus& bus) { return branch_if(bus, (regs_.cc & (CC_Z | CC_C)) != 0); }
uint8_t Cpu::op_bcc(Bus& bus) { return branch_if(bus, (regs_.cc & CC_C) == 0); }
uint8_t Cpu::op_bcs(Bus& bus) { return branch_if(bus, (regs_.cc & CC_C) != 0); }
uint8_t Cpu::op_bpl(Bus& bus) { return branch_if(bus, (regs_.cc & CC_N) == 0); }
uint8_t Cpu::op_bmi(Bus& bus) { return branch_if(bus, (regs_.cc & CC_N) != 0); }
uint8_t Cpu::op_bvc(Bus& bus) { return branch_if(bus, (regs_.cc & CC_V) == 0); }
uint8_t Cpu::op_bvs(Bus& bus) { return branch_if(bus, (regs_.cc & CC_V) != 0); }
uint8_t Cpu::op_bge(Bus& bus) { return branch_if(bus, ((regs_.cc & CC_N) >> 3) == ((regs_.cc & CC_V) >> 1)); }
uint8_t Cpu::op_blt(Bus& bus) { return branch_if(bus, ((regs_.cc & CC_N) >> 3) != ((regs_.cc & CC_V) >> 1)); }
uint8_t Cpu::op_bgt(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) == 0 &&
                                                  (((regs_.cc & CC_N) >> 3) == ((regs_.cc & CC_V) >> 1))); }
uint8_t Cpu::op_ble(Bus& bus) { return branch_if(bus, (regs_.cc & CC_Z) != 0 ||
                                                  (((regs_.cc & CC_N) >> 3) != ((regs_.cc & CC_V) >> 1))); }

// ----- Logical -----

uint8_t Cpu::op_anda_imm(Bus& bus) {
    regs_.a &= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_anda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_anda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_anda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a &= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_bita_imm(Bus& bus) {
    const uint8_t res = static_cast<uint8_t>(regs_.a & fetch_byte(bus));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 2;
}
uint8_t Cpu::op_bita_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 4;
}
uint8_t Cpu::op_bita_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_bita_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.a & read_byte(bus, pb.address));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_andb_imm(Bus& bus) {
    regs_.b &= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_andb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_andb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b &= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_andb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b &= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_bitb_imm(Bus& bus) {
    const uint8_t res = static_cast<uint8_t>(regs_.b & fetch_byte(bus));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 2;
}
uint8_t Cpu::op_bitb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 4;
}
uint8_t Cpu::op_bitb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, addr));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_bitb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = static_cast<uint8_t>(regs_.b & read_byte(bus, pb.address));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x80) regs_.cc |= CC_N;
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_ora_imm(Bus& bus) {
    regs_.a |= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_ora_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_ora_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_ora_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a |= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_orb_imm(Bus& bus) {
    regs_.b |= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_orb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_orb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b |= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_orb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b |= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_eora_imm(Bus& bus) {
    regs_.a ^= fetch_byte(bus);
    flags_logic8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_eora_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 4;
}
uint8_t Cpu::op_eora_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.a);
    return 5;
}
uint8_t Cpu::op_eora_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a ^= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.a);
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_eorb_imm(Bus& bus) {
    regs_.b ^= fetch_byte(bus);
    flags_logic8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_eorb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 4;
}
uint8_t Cpu::op_eorb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b ^= read_byte(bus, addr);
    flags_logic8(regs_, regs_.b);
    return 5;
}
uint8_t Cpu::op_eorb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b ^= read_byte(bus, pb.address);
    flags_logic8(regs_, regs_.b);
    return static_cast<uint8_t>(4 + pb.cycles);
}

// ----- Arithmetic -----

uint8_t Cpu::op_adca_imm(Bus& bus) {
    regs_.a = adc8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adca_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adca_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adca_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a = adc8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_adcb_imm(Bus& bus) {
    regs_.b = adc8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adcb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adcb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adcb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b = adc8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_sbca_imm(Bus& bus) {
    regs_.a = sbc8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_sbca_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_sbca_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_sbca_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a = sbc8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_sbcb_imm(Bus& bus) {
    regs_.b = sbc8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_sbcb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_sbcb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_sbcb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b = sbc8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_adda_imm(Bus& bus) {
    regs_.a = add8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_adda_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = add8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_adda_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = add8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_adda_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a = add8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_addb_imm(Bus& bus) {
    regs_.b = add8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_addb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = add8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_addb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = add8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_addb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b = add8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_suba_imm(Bus& bus) {
    regs_.a = sub8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_suba_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_suba_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_suba_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.a = sub8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_subb_imm(Bus& bus) {
    regs_.b = sub8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_subb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_subb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_subb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.b = sub8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_cmpa_imm(Bus& bus) {
    sub8(regs_, regs_.a, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_cmpa_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub8(regs_, regs_.a, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_cmpa_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub8(regs_, regs_.a, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_cmpa_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub8(regs_, regs_.a, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_cmpb_imm(Bus& bus) {
    sub8(regs_, regs_.b, fetch_byte(bus));
    return 2;
}
uint8_t Cpu::op_cmpb_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub8(regs_, regs_.b, read_byte(bus, addr));
    return 4;
}
uint8_t Cpu::op_cmpb_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub8(regs_, regs_.b, read_byte(bus, addr));
    return 5;
}
uint8_t Cpu::op_cmpb_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub8(regs_, regs_.b, read_byte(bus, pb.address));
    return static_cast<uint8_t>(4 + pb.cycles);
}

uint8_t Cpu::op_addd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 4;
}

uint8_t Cpu::op_addd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 6;
}

uint8_t Cpu::op_addd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}

uint8_t Cpu::op_addd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = add16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_subd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 4;
}

uint8_t Cpu::op_subd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 6;
}

uint8_t Cpu::op_subd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}

uint8_t Cpu::op_subd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = sub16(regs_, d, value);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_cmpd_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 5; // prefix opcode adds one cycle
}

uint8_t Cpu::op_cmpd_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 7;
}

uint8_t Cpu::op_cmpd_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint16_t value = static_cast<uint16_t>((read_byte(bus, addr) << 8) | read_byte(bus, static_cast<uint16_t>(addr + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return 8;
}

uint8_t Cpu::op_cmpd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t value =
        static_cast<uint16_t>((read_byte(bus, pb.address) << 8) | read_byte(bus, static_cast<uint16_t>(pb.address + 1)));
    const uint16_t d = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    sub16(regs_, d, value);
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmpx_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.x, value);
    return 4;
}
uint8_t Cpu::op_cmpx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.x, read_word(bus, addr));
    return 6;
}
uint8_t Cpu::op_cmpx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.x, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub16(regs_, regs_.x, read_word(bus, pb.address));
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_cmpy_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.y, value);
    return 5;
}
uint8_t Cpu::op_cmpy_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.y, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpy_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.y, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmpy_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub16(regs_, regs_.y, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmpu_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.u, value);
    return 5;
}
uint8_t Cpu::op_cmpu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.u, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmpu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.u, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmpu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub16(regs_, regs_.u, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_cmps_imm(Bus& bus) {
    const uint16_t value = fetch_word(bus);
    sub16(regs_, regs_.s, value);
    return 5;
}
uint8_t Cpu::op_cmps_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    sub16(regs_, regs_.s, read_word(bus, addr));
    return 7;
}
uint8_t Cpu::op_cmps_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    sub16(regs_, regs_.s, read_word(bus, addr));
    return 8;
}
uint8_t Cpu::op_cmps_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub16(regs_, regs_.s, read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

// ----- 16-bit loads/stores -----

uint8_t Cpu::op_ldx_imm(Bus& bus) {
    regs_.x = fetch_word(bus);
    set_flags_nz16(regs_.x);
    return 3;
}
uint8_t Cpu::op_ldx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.x = read_word(bus, addr);
    set_flags_nz16(regs_.x);
    return 5;
}
uint8_t Cpu::op_ldx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.x = read_word(bus, addr);
    set_flags_nz16(regs_.x);
    return 6;
}
uint8_t Cpu::op_ldx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.x = read_word(bus, pb.address);
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_ldy_imm(Bus& bus) {
    regs_.y = fetch_word(bus);
    set_flags_nz16(regs_.y);
    return 4;
}
uint8_t Cpu::op_ldy_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.y = read_word(bus, addr);
    set_flags_nz16(regs_.y);
    return 6;
}
uint8_t Cpu::op_ldy_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.y = read_word(bus, addr);
    set_flags_nz16(regs_.y);
    return 7;
}
uint8_t Cpu::op_ldy_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.y = read_word(bus, pb.address);
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_ldu_imm(Bus& bus) {
    regs_.u = fetch_word(bus);
    set_flags_nz16(regs_.u);
    return 3;
}
uint8_t Cpu::op_ldu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.u = read_word(bus, addr);
    set_flags_nz16(regs_.u);
    return 5;
}
uint8_t Cpu::op_ldu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.u = read_word(bus, addr);
    set_flags_nz16(regs_.u);
    return 6;
}
uint8_t Cpu::op_ldu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.u = read_word(bus, pb.address);
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_lds_imm(Bus& bus) {
    regs_.s = fetch_word(bus);
    set_flags_nz16(regs_.s);
    return 4;
}
uint8_t Cpu::op_lds_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    regs_.s = read_word(bus, addr);
    set_flags_nz16(regs_.s);
    return 6;
}
uint8_t Cpu::op_lds_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    regs_.s = read_word(bus, addr);
    set_flags_nz16(regs_.s);
    return 7;
}
uint8_t Cpu::op_lds_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.s = read_word(bus, pb.address);
    set_flags_nz16(regs_.s);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_stx_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.x);
    set_flags_nz16(regs_.x);
    return 5;
}
uint8_t Cpu::op_stx_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.x);
    set_flags_nz16(regs_.x);
    return 6;
}
uint8_t Cpu::op_stx_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, regs_.x);
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sty_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.y);
    set_flags_nz16(regs_.y);
    return 6;
}
uint8_t Cpu::op_sty_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.y);
    set_flags_nz16(regs_.y);
    return 7;
}
uint8_t Cpu::op_sty_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, regs_.y);
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_stu_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.u);
    set_flags_nz16(regs_.u);
    return 5;
}
uint8_t Cpu::op_stu_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.u);
    set_flags_nz16(regs_.u);
    return 6;
}
uint8_t Cpu::op_stu_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, regs_.u);
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(5 + pb.cycles);
}

uint8_t Cpu::op_sts_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, regs_.s);
    set_flags_nz16(regs_.s);
    return 6;
}
uint8_t Cpu::op_sts_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, regs_.s);
    set_flags_nz16(regs_.s);
    return 7;
}
uint8_t Cpu::op_sts_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, regs_.s);
    set_flags_nz16(regs_.s);
    return static_cast<uint8_t>(6 + pb.cycles);
}

// ----- LEA -----

uint8_t Cpu::op_leax(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.x = pb.address;
    set_flags_nz16(regs_.x);
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leay(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.y = pb.address;
    set_flags_nz16(regs_.y);
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leas(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.s = pb.address;
    return static_cast<uint8_t>(4 + pb.cycles);
}
uint8_t Cpu::op_leau(Bus& bus) {
    const auto pb = indexed_address(bus);
    regs_.u = pb.address;
    set_flags_nz16(regs_.u);
    return static_cast<uint8_t>(4 + pb.cycles);
}

// ----- Stack -----

uint8_t Cpu::op_pshs(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x80) { push_byte(bus, regs_.cc); ++count; }
    if (mask & 0x40) { push_byte(bus, regs_.a); ++count; }
    if (mask & 0x20) { push_byte(bus, regs_.b); ++count; }
    if (mask & 0x10) { push_byte(bus, regs_.dp); ++count; }
    if (mask & 0x08) { push_word(bus, regs_.x); count += 2; }
    if (mask & 0x04) { push_word(bus, regs_.y); count += 2; }
    if (mask & 0x02) { push_word(bus, regs_.u); count += 2; }
    if (mask & 0x01) { push_word(bus, regs_.pc); count += 2; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_puls(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x01) { regs_.pc = pull_word(bus); count += 2; }
    if (mask & 0x02) { regs_.u = pull_word(bus); count += 2; }
    if (mask & 0x04) { regs_.y = pull_word(bus); count += 2; }
    if (mask & 0x08) { regs_.x = pull_word(bus); count += 2; }
    if (mask & 0x10) { regs_.dp = pull_byte(bus); ++count; }
    if (mask & 0x20) { regs_.b = pull_byte(bus); ++count; }
    if (mask & 0x40) { regs_.a = pull_byte(bus); ++count; }
    if (mask & 0x80) { regs_.cc = pull_byte(bus); ++count; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_pshu(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x80) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.cc); ++count; }
    if (mask & 0x40) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.a); ++count; }
    if (mask & 0x20) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.b); ++count; }
    if (mask & 0x10) { regs_.u = static_cast<uint16_t>(regs_.u - 1); write_byte(bus, regs_.u, regs_.dp); ++count; }
    if (mask & 0x08) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.x)); write_byte(bus, regs_.u, lo(regs_.x)); count += 2; }
    if (mask & 0x04) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.y)); write_byte(bus, regs_.u, lo(regs_.y)); count += 2; }
    if (mask & 0x02) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.s)); write_byte(bus, regs_.u, lo(regs_.s)); count += 2; }
    if (mask & 0x01) { regs_.u = static_cast<uint16_t>(regs_.u - 2); write_byte(bus, static_cast<uint16_t>(regs_.u + 1), hi(regs_.pc)); write_byte(bus, regs_.u, lo(regs_.pc)); count += 2; }
    return static_cast<uint8_t>(5 + count);
}

uint8_t Cpu::op_pulu(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    uint8_t count = 0;
    if (mask & 0x01) { regs_.pc = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x02) { regs_.s = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x04) { regs_.y = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x08) { regs_.x = static_cast<uint16_t>((read_byte(bus, regs_.u + 1) << 8) | read_byte(bus, regs_.u)); regs_.u = static_cast<uint16_t>(regs_.u + 2); count += 2; }
    if (mask & 0x10) { regs_.dp = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x20) { regs_.b = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x40) { regs_.a = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    if (mask & 0x80) { regs_.cc = read_byte(bus, regs_.u); regs_.u = static_cast<uint16_t>(regs_.u + 1); ++count; }
    return static_cast<uint8_t>(5 + count);
}

// ----- Accumulator unary/shift -----

uint8_t Cpu::op_nega(Bus&) {
    regs_.a = neg8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_negb(Bus&) {
    regs_.b = neg8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_coma(Bus&) {
    regs_.a = com8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_comb(Bus&) {
    regs_.b = com8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_lsra(Bus&) {
    regs_.a = lsr8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_lsrb(Bus&) {
    regs_.b = lsr8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_rora(Bus&) {
    regs_.a = ror8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_rorb(Bus&) {
    regs_.b = ror8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_asra(Bus&) {
    regs_.a = asr8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_asrb(Bus&) {
    regs_.b = asr8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_asla(Bus&) {
    regs_.a = asl8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_aslb(Bus&) {
    regs_.b = asl8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_rola(Bus&) {
    regs_.a = rol8_op(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_rolb(Bus&) {
    regs_.b = rol8_op(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_deca(Bus&) {
    regs_.a = dec8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_decb(Bus&) {
    regs_.b = dec8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_inca(Bus&) {
    regs_.a = inc8(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_incb(Bus&) {
    regs_.b = inc8(regs_, regs_.b);
    return 2;
}
uint8_t Cpu::op_tsta(Bus&) {
    set_flags_tst(regs_, regs_.a);
    return 2;
}
uint8_t Cpu::op_tstb(Bus&) {
    set_flags_tst(regs_, regs_.b);
    return 2;
}

// ----- Memory unary/shift -----

uint8_t Cpu::op_neg_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = neg8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_neg_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = neg8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_neg_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = neg8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_com_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = com8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_com_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = com8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_com_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = com8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_lsr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = lsr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_lsr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = lsr8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_lsr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = lsr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_ror_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = ror8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_ror_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = ror8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_ror_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = ror8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_asr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = asr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_asr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = asr8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_asr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = asr8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_asl_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = asl8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_asl_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = asl8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_asl_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = asl8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_rol_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = rol8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_rol_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = rol8_op(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_rol_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = rol8_op(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_dec_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = dec8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_dec_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = dec8(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_dec_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = dec8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_inc_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t res = inc8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 6;
}
uint8_t Cpu::op_inc_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t res = inc8(regs_, read_byte(bus, pb.address));
    write_byte(bus, pb.address, res);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_inc_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t res = inc8(regs_, read_byte(bus, addr));
    write_byte(bus, addr, res);
    return 7;
}

uint8_t Cpu::op_tst_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint8_t val = read_byte(bus, addr);
    set_flags_tst(regs_, val);
    return 6;
}
uint8_t Cpu::op_tst_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint8_t val = read_byte(bus, pb.address);
    set_flags_tst(regs_, val);
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_tst_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint8_t val = read_byte(bus, addr);
    set_flags_tst(regs_, val);
    return 7;
}

uint8_t Cpu::op_clr_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_byte(bus, addr, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 6;
}
uint8_t Cpu::op_clr_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_byte(bus, pb.address, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_clr_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_byte(bus, addr, 0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 7;
}

// ----- Misc -----

uint8_t Cpu::op_abx(Bus&) {
    regs_.x = static_cast<uint16_t>(regs_.x + regs_.b);
    return 3;
}

uint8_t Cpu::op_sex(Bus&) {
    regs_.b = (regs_.a & 0x80) ? 0xFF : 0x00;
    set_flags_nz16(static_cast<uint16_t>((regs_.a << 8) | regs_.b));
    return 2;
}

uint8_t Cpu::op_andcc(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc &= mask;
    return 3;
}

uint8_t Cpu::op_orcc(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    regs_.cc |= mask;
    return 3;
}

uint8_t Cpu::op_daa(Bus&) {
    uint8_t adjust = 0;
    bool carry = (regs_.cc & CC_C) != 0;
    if ((regs_.a & 0x0F) > 9 || (regs_.cc & CC_H)) {
        adjust |= 0x06;
    }
    if ((regs_.a > 0x99) || carry) {
        adjust |= 0x60;
        carry = true;
    }
    const uint8_t result = static_cast<uint8_t>(regs_.a + adjust);
    regs_.a = result;
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (result & 0x80) regs_.cc |= CC_N;
    if (result == 0) regs_.cc |= CC_Z;
    if (carry) regs_.cc |= CC_C;
    return 2;
}
uint8_t Cpu::op_tfr(Bus& bus) {
    (void)bus;
    const uint8_t post = fetch_byte(bus);
    const uint8_t src = post >> 4;
    const uint8_t dst = post & 0x0F;

    uint16_t value16{};
    if (get_reg16_by_code(src, value16)) {
        set_reg16_by_code(dst, value16);
        return 6;
    }
    if (auto* s8 = reg8_by_code(src)) {
        if (auto* d8 = reg8_by_code(dst)) {
            *d8 = *s8;
        }
        return 6;
    }
    return 6;
}

uint8_t Cpu::op_exg(Bus& bus) {
    (void)bus;
    const uint8_t post = fetch_byte(bus);
    const uint8_t r1c = post >> 4;
    const uint8_t r2c = post & 0x0F;

    uint16_t v1{}, v2{};
    if (get_reg16_by_code(r1c, v1) && get_reg16_by_code(r2c, v2)) {
        set_reg16_by_code(r1c, v2);
        set_reg16_by_code(r2c, v1);
        return 8;
    }

    if (auto* r1 = reg8_by_code(r1c)) {
        if (auto* r2 = reg8_by_code(r2c)) {
            std::swap(*r1, *r2);
        }
    }
    return 8;
}

// ---- 6309 extensions ----

uint8_t Cpu::op_ldq_imm(Bus& bus) {
    const uint32_t val = (static_cast<uint32_t>(fetch_word(bus)) << 16) | fetch_word(bus);
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_ldq_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, addr)) << 16) |
                         read_word(bus, static_cast<uint16_t>(addr + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_ldq_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, addr)) << 16) |
                         read_word(bus, static_cast<uint16_t>(addr + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return 9;
}
uint8_t Cpu::op_ldq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint32_t val = (static_cast<uint32_t>(read_word(bus, pb.address)) << 16) |
                         read_word(bus, static_cast<uint16_t>(pb.address + 2));
    set_reg_q(val);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (val == 0) regs_.cc |= CC_Z;
    if (val & 0x80000000u) regs_.cc |= CC_N;
    return static_cast<uint8_t>(8 + pb.cycles);
}
uint8_t Cpu::op_stq_dir(Bus& bus) {
    const uint16_t addr = direct_address(bus);
    write_word(bus, addr, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(addr + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_stq_ext(Bus& bus) {
    const uint16_t addr = extended_address(bus);
    write_word(bus, addr, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(addr + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return 9;
}
uint8_t Cpu::op_stq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, static_cast<uint16_t>((reg_q() >> 16) & 0xFFFF));
    write_word(bus, static_cast<uint16_t>(pb.address + 2), static_cast<uint16_t>(reg_q() & 0xFFFF));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (reg_q() == 0) regs_.cc |= CC_Z;
    if (reg_q() & 0x80000000u) regs_.cc |= CC_N;
    return static_cast<uint8_t>(8 + pb.cycles);
}

uint8_t Cpu::op_ldw_imm(Bus& bus) {
    set_reg_w(fetch_word(bus));
    set_flags_nz16(reg_w());
    return 4;
}
uint8_t Cpu::op_ldw_dir(Bus& bus) {
    set_reg_w(read_word(bus, direct_address(bus)));
    set_flags_nz16(reg_w());
    return 6;
}
uint8_t Cpu::op_ldw_ext(Bus& bus) {
    set_reg_w(read_word(bus, extended_address(bus)));
    set_flags_nz16(reg_w());
    return 7;
}
uint8_t Cpu::op_ldw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    set_reg_w(read_word(bus, pb.address));
    set_flags_nz16(reg_w());
    return static_cast<uint8_t>(6 + pb.cycles);
}
uint8_t Cpu::op_stw_dir(Bus& bus) {
    write_word(bus, direct_address(bus), reg_w());
    set_flags_nz16(reg_w());
    return 6;
}
uint8_t Cpu::op_stw_ext(Bus& bus) {
    write_word(bus, extended_address(bus), reg_w());
    set_flags_nz16(reg_w());
    return 7;
}
uint8_t Cpu::op_stw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    write_word(bus, pb.address, reg_w());
    set_flags_nz16(reg_w());
    return static_cast<uint8_t>(6 + pb.cycles);
}

uint8_t Cpu::op_addw_imm(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), fetch_word(bus)));
    return 5;
}
uint8_t Cpu::op_addw_dir(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), read_word(bus, direct_address(bus))));
    return 7;
}
uint8_t Cpu::op_addw_ext(Bus& bus) {
    set_reg_w(add16(regs_, reg_w(), read_word(bus, extended_address(bus))));
    return 8;
}
uint8_t Cpu::op_addw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    set_reg_w(add16(regs_, reg_w(), read_word(bus, pb.address)));
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_subw_imm(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), fetch_word(bus)));
    return 5;
}
uint8_t Cpu::op_subw_dir(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, direct_address(bus))));
    return 7;
}
uint8_t Cpu::op_subw_ext(Bus& bus) {
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, extended_address(bus))));
    return 8;
}
uint8_t Cpu::op_subw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    set_reg_w(sub16(regs_, reg_w(), read_word(bus, pb.address)));
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_cmpw_imm(Bus& bus) {
    sub16(regs_, reg_w(), fetch_word(bus));
    return 5;
}
uint8_t Cpu::op_cmpw_dir(Bus& bus) {
    sub16(regs_, reg_w(), read_word(bus, direct_address(bus)));
    return 7;
}
uint8_t Cpu::op_cmpw_ext(Bus& bus) {
    sub16(regs_, reg_w(), read_word(bus, extended_address(bus)));
    return 8;
}
uint8_t Cpu::op_cmpw_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    sub16(regs_, reg_w(), read_word(bus, pb.address));
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_incw_inh(Bus&) {
    const uint16_t v = static_cast<uint16_t>(reg_w() + 1);
    set_reg_w(v);
    set_flags_nz16(v);
    return 3;
}
uint8_t Cpu::op_decw_inh(Bus&) {
    const uint16_t v = reg_w() - 1;
    set_reg_w(v);
    set_flags_nz16(v);
    return 3;
}
uint8_t Cpu::op_tstw_inh(Bus&) {
    set_flags_nz16(reg_w());
    return 3;
}
uint8_t Cpu::op_clrw_inh(Bus&) {
    set_reg_w(0);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
    regs_.cc |= CC_Z;
    return 3;
}

uint8_t Cpu::op_adde_imm(Bus& bus) { regs_.e = add8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_adde_dir(Bus& bus) { regs_.e = add8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_adde_ext(Bus& bus) { regs_.e = add8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_adde_idx(Bus& bus) { const auto pb = indexed_address(bus); regs_.e = add8(regs_, regs_.e, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }
uint8_t Cpu::op_addf_imm(Bus& bus) { regs_.f = add8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_addf_dir(Bus& bus) { regs_.f = add8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_addf_ext(Bus& bus) { regs_.f = add8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_addf_idx(Bus& bus) { const auto pb = indexed_address(bus); regs_.f = add8(regs_, regs_.f, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }
uint8_t Cpu::op_sube_imm(Bus& bus) { regs_.e = sub8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_sube_dir(Bus& bus) { regs_.e = sub8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_sube_ext(Bus& bus) { regs_.e = sub8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_sube_idx(Bus& bus) { const auto pb = indexed_address(bus); regs_.e = sub8(regs_, regs_.e, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }
uint8_t Cpu::op_subf_imm(Bus& bus) { regs_.f = sub8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_subf_dir(Bus& bus) { regs_.f = sub8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_subf_ext(Bus& bus) { regs_.f = sub8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_subf_idx(Bus& bus) { const auto pb = indexed_address(bus); regs_.f = sub8(regs_, regs_.f, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }
uint8_t Cpu::op_cmpe_imm(Bus& bus) { sub8(regs_, regs_.e, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_cmpe_dir(Bus& bus) { sub8(regs_, regs_.e, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_cmpe_ext(Bus& bus) { sub8(regs_, regs_.e, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_cmpe_idx(Bus& bus) { const auto pb = indexed_address(bus); sub8(regs_, regs_.e, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }
uint8_t Cpu::op_cmpf_imm(Bus& bus) { sub8(regs_, regs_.f, fetch_byte(bus)); return 3; }
uint8_t Cpu::op_cmpf_dir(Bus& bus) { sub8(regs_, regs_.f, read_byte(bus, direct_address(bus))); return 5; }
uint8_t Cpu::op_cmpf_ext(Bus& bus) { sub8(regs_, regs_.f, read_byte(bus, extended_address(bus))); return 6; }
uint8_t Cpu::op_cmpf_idx(Bus& bus) { const auto pb = indexed_address(bus); sub8(regs_, regs_.f, read_byte(bus, pb.address)); return static_cast<uint8_t>(5 + pb.cycles); }

uint8_t Cpu::op_adcd_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}
uint8_t Cpu::op_adcd_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 5;
}
uint8_t Cpu::op_adcd_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 8;
}
uint8_t Cpu::op_adcd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = add16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_sbcd_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 5;
}
uint8_t Cpu::op_sbcd_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 7;
}
uint8_t Cpu::op_sbcd_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return 8;
}
uint8_t Cpu::op_sbcd_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = sub16_carry(regs_, static_cast<uint16_t>((regs_.a << 8) | regs_.b), val, (regs_.cc & CC_C) != 0);
    regs_.a = hi(res);
    regs_.b = lo(res);
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_ord_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 7;
}
uint8_t Cpu::op_ord_imm(Bus& bus) {
    const uint16_t val = fetch_word(bus);
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 5;
}
uint8_t Cpu::op_ord_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return 8;
}
uint8_t Cpu::op_ord_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t val = read_word(bus, pb.address);
    const uint16_t res = static_cast<uint16_t>(((regs_.a << 8) | regs_.b) | val);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res == 0) regs_.cc |= CC_Z;
    if (res & 0x8000) regs_.cc |= CC_N;
    return static_cast<uint8_t>(7 + pb.cycles);
}

uint8_t Cpu::op_divq_imm(Bus& bus) {
    const uint16_t divisor = fetch_word(bus);
    const uint32_t dividend = reg_q();
    if (divisor == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 34;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(divisor);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 34;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 34;
}
uint8_t Cpu::op_divq_dir(Bus& bus) {
    const uint16_t val = read_word(bus, direct_address(bus));
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 36;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 36;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 36;
}
uint8_t Cpu::op_divq_ext(Bus& bus) {
    const uint16_t val = read_word(bus, extended_address(bus));
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return 37;
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return 37;
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return 37;
}
uint8_t Cpu::op_divq_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const uint16_t val = read_word(bus, pb.address);
    const uint32_t dividend = reg_q();
    if (val == 0) {
        regs_.md |= 0x80;
        regs_.pc = read_word(bus, 0xFFF0);
        return static_cast<uint8_t>(36 + pb.cycles);
    }
    const int32_t s_dividend = static_cast<int32_t>(dividend);
    const int32_t s_divisor = static_cast<int16_t>(val);
    const int32_t q = s_dividend / s_divisor;
    const int32_t r = s_dividend % s_divisor;
    if (q < -32768 || q > 32767) {
        regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_C));
        regs_.cc |= CC_V;
        return static_cast<uint8_t>(36 + pb.cycles);
    }
    set_reg_w(static_cast<uint16_t>(q & 0xFFFF));
    const uint16_t rem = static_cast<uint16_t>(r & 0xFFFF);
    regs_.a = hi(rem);
    regs_.b = lo(rem);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x8000) regs_.cc |= CC_N;
    if (q & 0x1) regs_.cc |= CC_C;
    return static_cast<uint8_t>(37 + pb.cycles);
}

uint8_t Cpu::op_muld_imm(Bus& bus) {
    const int16_t m = static_cast<int16_t>(fetch_word(bus));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 28;
}
uint8_t Cpu::op_muld_dir(Bus& bus) {
    const int16_t m = static_cast<int16_t>(read_word(bus, direct_address(bus)));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 30;
}
uint8_t Cpu::op_muld_ext(Bus& bus) {
    const int16_t m = static_cast<int16_t>(read_word(bus, extended_address(bus)));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return 31;
}
uint8_t Cpu::op_muld_idx(Bus& bus) {
    const auto pb = indexed_address(bus);
    const int16_t m = static_cast<int16_t>(read_word(bus, pb.address));
    const int32_t prod = static_cast<int16_t>((regs_.a << 8) | regs_.b) * m;
    set_reg_q(static_cast<uint32_t>(prod));
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (prod == 0) regs_.cc |= CC_Z;
    if (prod & 0x80000000) regs_.cc |= CC_N;
    return static_cast<uint8_t>(31 + pb.cycles);
}

uint8_t Cpu::op_addr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = dest16 ? add16(regs_, d, s) : add8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_subr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = dest16 ? sub16(regs_, d, s) : sub8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_cmpr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    if (dest16) sub16(regs_, d, s); else sub8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    return 4;
}
uint8_t Cpu::op_sbcr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    uint16_t res = dest16 ? sub16_carry(regs_, d, s, (regs_.cc & CC_C) != 0)
                          : sbc8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_adcr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    uint16_t res = dest16 ? add16_carry(regs_, d, s, (regs_.cc & CC_C) != 0)
                          : adc8(regs_, static_cast<uint8_t>(d & 0xFF), static_cast<uint8_t>(s & 0xFF));
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}
uint8_t Cpu::op_orr(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t dst = post & 0x0F;
    const uint8_t src = post >> 4;
    const bool dest16 = reg_is_16bit(dst);
    const uint16_t s = read_reg_for_dest(regs_, src, dest16);
    const uint16_t d = read_reg_for_dest(regs_, dst, dest16);
    const uint16_t res = static_cast<uint16_t>(d | s);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
    if (dest16) {
        if (res & 0x8000) regs_.cc |= CC_N;
        if (res == 0) regs_.cc |= CC_Z;
    } else {
        if (res & 0x80) regs_.cc |= CC_N;
        if ((res & 0xFF) == 0) regs_.cc |= CC_Z;
    }
    write_reg_sized(*this, dst, res, dest16);
    return 4;
}

uint8_t Cpu::op_lsl_d(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>(val << 1);
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rold(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>((val << 1) | ((regs_.cc & CC_C) ? 1 : 0));
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rord(Bus&) {
    const uint16_t val = static_cast<uint16_t>((regs_.a << 8) | regs_.b);
    const uint16_t res = static_cast<uint16_t>((val >> 1) | ((regs_.cc & CC_C) ? 0x8000 : 0));
    regs_.a = hi(res);
    regs_.b = lo(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x1) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_lslw_inh(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>(val << 1);
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rolw(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>((val << 1) | ((regs_.cc & CC_C) ? 1 : 0));
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x8000) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}
uint8_t Cpu::op_rorw(Bus&) {
    const uint16_t val = reg_w();
    const uint16_t res = static_cast<uint16_t>((val >> 1) | ((regs_.cc & CC_C) ? 0x8000 : 0));
    set_reg_w(res);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
    if (res & 0x8000) regs_.cc |= CC_N;
    if (res == 0) regs_.cc |= CC_Z;
    if (val & 0x1) regs_.cc |= CC_C;
    if (((res ^ (regs_.cc & CC_C ? 0x8000 : 0)) & 0x8000) != 0) regs_.cc |= CC_V;
    return 3;
}

uint8_t Cpu::op_ldmd(Bus& bus) {
    regs_.md = fetch_byte(bus);
    return 5;
}

uint8_t Cpu::op_sexw(Bus&) {
    const uint16_t w = reg_w();
    const uint32_t q = (static_cast<int32_t>(static_cast<int16_t>(w)) << 16) | static_cast<uint16_t>(w);
    set_reg_q(q);
    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
    if (q == 0) regs_.cc |= CC_Z;
    if (q & 0x80000000u) regs_.cc |= CC_N;
    return 4;
}

uint8_t Cpu::op_tfm_common(Bus& bus, bool inc_src, bool inc_dst) {
    const uint8_t post = fetch_byte(bus);
    const uint8_t src_code = post >> 4;
    const uint8_t dst_code = post & 0x0F;
    // Only X,Y,U,S,D are valid
    auto reg_ptr = [&](uint8_t code) -> uint16_t* {
        switch (code & 0x0F) {
        case 0x00: return nullptr; // D not addressable as pointer here
        case 0x01: return &regs_.x;
        case 0x02: return &regs_.y;
        case 0x03: return &regs_.u;
        case 0x04: return &regs_.s;
        default: return nullptr;
        }
    };
    uint16_t* src_ptr = reg_ptr(src_code);
    uint16_t* dst_ptr = reg_ptr(dst_code);
    if (!src_ptr || !dst_ptr) {
        regs_.pc = read_word(bus, 0xFFF0);
        return 6;
    }
    uint32_t count = reg_w();
    if (count == 0) count = 0x10000;
    for (uint32_t i = 0; i < count; ++i) {
        const uint8_t byte = read_byte(bus, *src_ptr);
        if (inc_src) *src_ptr = static_cast<uint16_t>(*src_ptr + 1);
        else *src_ptr = static_cast<uint16_t>(*src_ptr - 1);
        write_byte(bus, *dst_ptr, byte);
        if (inc_dst) *dst_ptr = static_cast<uint16_t>(*dst_ptr + 1);
        else *dst_ptr = static_cast<uint16_t>(*dst_ptr - 1);
        set_reg_w(static_cast<uint16_t>(reg_w() - 1));
        if (reg_w() == 0) break;
    }
    return 6; // base cycles; ignores per-byte timing.
}

uint8_t Cpu::op_tfm_pp(Bus& bus) { return op_tfm_common(bus, true, true); }
uint8_t Cpu::op_tfm_mm(Bus& bus) { return op_tfm_common(bus, false, false); }
uint8_t Cpu::op_tfm_pn(Bus& bus) { return op_tfm_common(bus, true, false); }
uint8_t Cpu::op_tfm_np(Bus& bus) { return op_tfm_common(bus, false, true); }

uint8_t Cpu::op_aim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_aim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    const uint8_t res = read_byte(bus, pb.address) & mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_aim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_eim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) ^ mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_eim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    const uint8_t res = read_byte(bus, pb.address) ^ mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_eim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) ^ mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_oim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) | mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_oim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    const uint8_t res = read_byte(bus, pb.address) | mask;
    write_byte(bus, pb.address, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_oim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) | mask;
    write_byte(bus, addr, res);
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}
uint8_t Cpu::op_tim_dir(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 6;
}
uint8_t Cpu::op_tim_idx(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const auto pb = indexed_address(bus);
    const uint8_t res = read_byte(bus, pb.address) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return static_cast<uint8_t>(7 + pb.cycles);
}
uint8_t Cpu::op_tim_ext(Bus& bus) {
    const uint8_t mask = fetch_byte(bus);
    const uint16_t addr = extended_address(bus);
    const uint8_t res = read_byte(bus, addr) & mask;
    set_flags_nz8(res);
    regs_.cc &= static_cast<uint8_t>(~CC_V);
    return 7;
}

static inline uint8_t* bitop_reg_ptr(Registers& regs, uint8_t reg_code) {
    switch ((reg_code >> 6) & 0x03) {
    case 0: return &regs.cc;
    case 1: return &regs.a;
    case 2: return &regs.b;
    default: return nullptr;
    }
}

uint8_t Cpu::op_band(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit & mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_biand(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit & static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit | mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bior(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit | static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_beor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit ^ mem_bit);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_bieor(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    uint8_t mem = read_byte(bus, addr);
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    const uint8_t mem_bit = (mem >> dst_bit) & 0x01;
    const uint8_t new_bit = static_cast<uint8_t>(reg_bit ^ static_cast<uint8_t>(mem_bit ^ 0x01));
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (new_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 7;
}
uint8_t Cpu::op_ldbt(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 7; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    const uint8_t mem = read_byte(bus, addr);
    const uint8_t mem_bit = (mem >> src_bit) & 0x01;
    *regp = static_cast<uint8_t>((*regp & static_cast<uint8_t>(~(1u << dst_bit))) | (mem_bit << dst_bit));
    return 7;
}
uint8_t Cpu::op_stbt(Bus& bus) {
    const uint8_t post = fetch_byte(bus);
    const uint16_t addr = direct_address(bus);
    uint8_t* regp = bitop_reg_ptr(regs_, post);
    if (!regp) { regs_.pc = read_word(bus, 0xFFF0); return 8; }
    const uint8_t src_bit = (post >> 3) & 0x07;
    const uint8_t dst_bit = post & 0x07;
    const uint8_t reg_bit = (*regp >> src_bit) & 0x01;
    uint8_t mem = read_byte(bus, addr);
    mem = static_cast<uint8_t>((mem & static_cast<uint8_t>(~(1u << dst_bit))) | (reg_bit << dst_bit));
    write_byte(bus, addr, mem);
    return 8;
}

} // namespace microlind
