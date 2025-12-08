#pragma once

#include <cstdint>
#include <functional>

namespace microlind {

class Bus;
class Cpu;
struct Registers;
uint16_t read_reg_for_dest(const Registers& regs, uint8_t src_code, bool dest_is_16);
void write_reg_sized(Cpu& cpu, uint8_t dest_code, uint16_t value, bool dest_is_16);

// Condition Code register bits (6809/6309 compatible).
enum : uint8_t {
    CC_C = 0x01,
    CC_V = 0x02,
    CC_Z = 0x04,
    CC_N = 0x08,
    CC_I = 0x10,
    CC_H = 0x20,
    CC_F = 0x40,
    CC_E = 0x80,
};

enum class CpuMode {
    MC6809,
    HD6309,
};

struct CpuTickResult {
    uint32_t cycles{};
};

struct Registers {
    uint8_t a{};
    uint8_t b{};
    uint8_t e{}; // 6309
    uint8_t f{}; // 6309
    uint8_t dp{};
    uint8_t cc{};
    uint8_t md{}; // 6309 mode register
    uint16_t x{};
    uint16_t y{};
    uint16_t u{};
    uint16_t s{};
    uint16_t pc{};
};

class Cpu {
public:
    explicit Cpu(CpuMode mode);

    [[nodiscard]] CpuMode mode() const { return mode_; }
    CpuTickResult tick(Bus& bus);
    Registers& regs() { return regs_; }
    const Registers& regs() const { return regs_; }
    void set_pc(uint16_t pc) { regs_.pc = pc; }
    [[nodiscard]] uint16_t last_pc() const { return last_pc_; }
    [[nodiscard]] uint8_t last_opcode() const { return last_opcode_; }
    [[nodiscard]] uint8_t last_prefix() const { return last_prefix_; }

private:
    friend uint16_t read_reg_for_dest(const Registers& regs, uint8_t src_code, bool dest_is_16);
    friend void write_reg_sized(Cpu& cpu, uint8_t dest_code, uint16_t value, bool dest_is_16);
    using Handler = uint8_t (Cpu::*)(Bus&);

    enum class IndexReg : uint8_t { X = 0, Y, U, S, PC };

    struct PostbyteResult {
        uint16_t address{};
        uint8_t cycles{};
    };

    uint8_t fetch_byte(Bus& bus);
    uint16_t fetch_word(Bus& bus);
    uint8_t read_byte(Bus& bus, uint16_t address);
    void write_byte(Bus& bus, uint16_t address, uint8_t value);

    uint16_t direct_address(Bus& bus);
    uint16_t extended_address(Bus& bus);
    PostbyteResult indexed_address(Bus& bus);

    void push_byte(Bus& bus, uint8_t value);
    void push_word(Bus& bus, uint16_t value);
    uint8_t pull_byte(Bus& bus);
    uint16_t pull_word(Bus& bus);

    void set_flags_nz8(uint8_t value);
    void set_flags_nz16(uint16_t value);
    uint8_t branch_if(Bus& bus, bool take);

    uint16_t& index_ref(IndexReg reg);
    uint16_t index_value(IndexReg reg) const;
    uint8_t* reg8_by_code(uint8_t code);
    bool get_reg16_by_code(uint8_t code, uint16_t& out) const;
    bool set_reg16_by_code(uint8_t code, uint16_t value);
    bool set_reg32_by_code(uint8_t code, uint32_t value);
    uint16_t reg_w() const;
    void set_reg_w(uint16_t value);
    uint32_t reg_q() const;
    void set_reg_q(uint32_t value);
    uint8_t op_tfm_common(Bus& bus, bool inc_src, bool inc_dst);

    // Instruction handlers (return cycles consumed).
    uint8_t op_nop(Bus&);
    uint8_t op_clra(Bus&);
    uint8_t op_clrb(Bus&);

    uint8_t op_lda_imm(Bus&);
    uint8_t op_lda_dir(Bus&);
    uint8_t op_lda_ext(Bus&);
    uint8_t op_lda_idx(Bus&);

    uint8_t op_ldb_imm(Bus&);
    uint8_t op_ldb_dir(Bus&);
    uint8_t op_ldb_ext(Bus&);
    uint8_t op_ldb_idx(Bus&);

    uint8_t op_ldd_imm(Bus&);
    uint8_t op_ldd_dir(Bus&);
    uint8_t op_ldd_ext(Bus&);
    uint8_t op_ldd_idx(Bus&);

    uint8_t op_sta_dir(Bus&);
    uint8_t op_sta_ext(Bus&);
    uint8_t op_stb_dir(Bus&);
    uint8_t op_stb_ext(Bus&);
    uint8_t op_std_dir(Bus&);
    uint8_t op_std_ext(Bus&);
    uint8_t op_sta_idx(Bus&);
    uint8_t op_stb_idx(Bus&);
    uint8_t op_std_idx(Bus&);

    uint8_t op_jmp_dir(Bus&);
    uint8_t op_jmp_ext(Bus&);
    uint8_t op_jmp_idx(Bus&);
    uint8_t op_jsr_dir(Bus&);
    uint8_t op_jsr_ext(Bus&);
    uint8_t op_jsr_idx(Bus&);
    uint8_t op_rts(Bus&);
    uint8_t op_rti(Bus&);
    uint8_t op_swi(Bus&);
    uint8_t op_swi2(Bus&);
    uint8_t op_swi3(Bus&);
    uint8_t op_cwai(Bus&);
    uint8_t op_sync(Bus&);
    uint8_t op_mul(Bus&);

    uint8_t op_bra(Bus&);
    uint8_t op_bsr(Bus&);
    uint8_t op_beq(Bus&);
    uint8_t op_bne(Bus&);

    uint8_t op_tfr(Bus&);
    uint8_t op_exg(Bus&);

    // Arithmetic / logic
    uint8_t op_anda_imm(Bus&);
    uint8_t op_anda_dir(Bus&);
    uint8_t op_anda_ext(Bus&);
    uint8_t op_anda_idx(Bus&);
    uint8_t op_andb_imm(Bus&);
    uint8_t op_andb_dir(Bus&);
    uint8_t op_andb_ext(Bus&);
    uint8_t op_andb_idx(Bus&);

    uint8_t op_ora_imm(Bus&);
    uint8_t op_ora_dir(Bus&);
    uint8_t op_ora_ext(Bus&);
    uint8_t op_ora_idx(Bus&);
    uint8_t op_orb_imm(Bus&);
    uint8_t op_orb_dir(Bus&);
    uint8_t op_orb_ext(Bus&);
    uint8_t op_orb_idx(Bus&);

    uint8_t op_eora_imm(Bus&);
    uint8_t op_eora_dir(Bus&);
    uint8_t op_eora_ext(Bus&);
    uint8_t op_eora_idx(Bus&);
    uint8_t op_eorb_imm(Bus&);
    uint8_t op_eorb_dir(Bus&);
    uint8_t op_eorb_ext(Bus&);
    uint8_t op_eorb_idx(Bus&);

    uint8_t op_adda_imm(Bus&);
    uint8_t op_adda_dir(Bus&);
    uint8_t op_adda_ext(Bus&);
    uint8_t op_adda_idx(Bus&);
    uint8_t op_addb_imm(Bus&);
    uint8_t op_addb_dir(Bus&);
    uint8_t op_addb_ext(Bus&);
    uint8_t op_addb_idx(Bus&);

    uint8_t op_suba_imm(Bus&);
    uint8_t op_suba_dir(Bus&);
    uint8_t op_suba_ext(Bus&);
    uint8_t op_suba_idx(Bus&);
    uint8_t op_subb_imm(Bus&);
    uint8_t op_subb_dir(Bus&);
    uint8_t op_subb_ext(Bus&);
    uint8_t op_subb_idx(Bus&);

    uint8_t op_cmpa_imm(Bus&);
    uint8_t op_cmpa_dir(Bus&);
    uint8_t op_cmpa_ext(Bus&);
    uint8_t op_cmpa_idx(Bus&);
    uint8_t op_cmpb_imm(Bus&);
    uint8_t op_cmpb_dir(Bus&);
    uint8_t op_cmpb_ext(Bus&);
    uint8_t op_cmpb_idx(Bus&);

    uint8_t op_leax(Bus&);
    uint8_t op_leay(Bus&);
    uint8_t op_leas(Bus&);
    uint8_t op_leau(Bus&);

    uint8_t op_pshs(Bus&);
    uint8_t op_puls(Bus&);
    uint8_t op_pshu(Bus&);
    uint8_t op_pulu(Bus&);

    // 16-bit arithmetic
    uint8_t op_addd_imm(Bus&);
    uint8_t op_addd_dir(Bus&);
    uint8_t op_addd_ext(Bus&);
    uint8_t op_addd_idx(Bus&);

    uint8_t op_subd_imm(Bus&);
    uint8_t op_subd_dir(Bus&);
    uint8_t op_subd_ext(Bus&);
    uint8_t op_subd_idx(Bus&);

    uint8_t op_cmpd_imm(Bus&);
    uint8_t op_cmpd_dir(Bus&);
    uint8_t op_cmpd_ext(Bus&);
    uint8_t op_cmpd_idx(Bus&);

    // Accumulator unary/shift
    uint8_t op_nega(Bus&);
    uint8_t op_negb(Bus&);
    uint8_t op_coma(Bus&);
    uint8_t op_comb(Bus&);
    uint8_t op_lsra(Bus&);
    uint8_t op_lsrb(Bus&);
    uint8_t op_rora(Bus&);
    uint8_t op_rorb(Bus&);
    uint8_t op_asra(Bus&);
    uint8_t op_asrb(Bus&);
    uint8_t op_asla(Bus&);
    uint8_t op_aslb(Bus&);
    uint8_t op_rola(Bus&);
    uint8_t op_rolb(Bus&);
    uint8_t op_deca(Bus&);
    uint8_t op_decb(Bus&);
    uint8_t op_inca(Bus&);
    uint8_t op_incb(Bus&);
    uint8_t op_tsta(Bus&);
    uint8_t op_tstb(Bus&);

    // Memory unary/shift
    uint8_t op_neg_dir(Bus&);
    uint8_t op_neg_idx(Bus&);
    uint8_t op_neg_ext(Bus&);
    uint8_t op_com_dir(Bus&);
    uint8_t op_com_idx(Bus&);
    uint8_t op_com_ext(Bus&);
    uint8_t op_lsr_dir(Bus&);
    uint8_t op_lsr_idx(Bus&);
    uint8_t op_lsr_ext(Bus&);
    uint8_t op_ror_dir(Bus&);
    uint8_t op_ror_idx(Bus&);
    uint8_t op_ror_ext(Bus&);
    uint8_t op_asr_dir(Bus&);
    uint8_t op_asr_idx(Bus&);
    uint8_t op_asr_ext(Bus&);
    uint8_t op_asl_dir(Bus&);
    uint8_t op_asl_idx(Bus&);
    uint8_t op_asl_ext(Bus&);
    uint8_t op_rol_dir(Bus&);
    uint8_t op_rol_idx(Bus&);
    uint8_t op_rol_ext(Bus&);
    uint8_t op_dec_dir(Bus&);
    uint8_t op_dec_idx(Bus&);
    uint8_t op_dec_ext(Bus&);
    uint8_t op_inc_dir(Bus&);
    uint8_t op_inc_idx(Bus&);
    uint8_t op_inc_ext(Bus&);
    uint8_t op_tst_dir(Bus&);
    uint8_t op_tst_idx(Bus&);
    uint8_t op_tst_ext(Bus&);
    uint8_t op_clr_dir(Bus&);
    uint8_t op_clr_idx(Bus&);
    uint8_t op_clr_ext(Bus&);

    // Additional arithmetic/logic
    uint8_t op_adca_imm(Bus&);
    uint8_t op_adca_dir(Bus&);
    uint8_t op_adca_ext(Bus&);
    uint8_t op_adca_idx(Bus&);
    uint8_t op_adcb_imm(Bus&);
    uint8_t op_adcb_dir(Bus&);
    uint8_t op_adcb_ext(Bus&);
    uint8_t op_adcb_idx(Bus&);

    uint8_t op_sbca_imm(Bus&);
    uint8_t op_sbca_dir(Bus&);
    uint8_t op_sbca_ext(Bus&);
    uint8_t op_sbca_idx(Bus&);
    uint8_t op_sbcb_imm(Bus&);
    uint8_t op_sbcb_dir(Bus&);
    uint8_t op_sbcb_ext(Bus&);
    uint8_t op_sbcb_idx(Bus&);

    uint8_t op_bita_imm(Bus&);
    uint8_t op_bita_dir(Bus&);
    uint8_t op_bita_ext(Bus&);
    uint8_t op_bita_idx(Bus&);
    uint8_t op_bitb_imm(Bus&);
    uint8_t op_bitb_dir(Bus&);
    uint8_t op_bitb_ext(Bus&);
    uint8_t op_bitb_idx(Bus&);

    // Branches (more conditions)
    uint8_t op_bhi(Bus&);
    uint8_t op_bls(Bus&);
    uint8_t op_bcc(Bus&);
    uint8_t op_bcs(Bus&);
    uint8_t op_bpl(Bus&);
    uint8_t op_bmi(Bus&);
    uint8_t op_bvc(Bus&);
    uint8_t op_bvs(Bus&);
    uint8_t op_bge(Bus&);
    uint8_t op_blt(Bus&);
    uint8_t op_bgt(Bus&);
    uint8_t op_ble(Bus&);

    // Loads / stores for 16-bit regs
    uint8_t op_ldx_imm(Bus&);
    uint8_t op_ldx_dir(Bus&);
    uint8_t op_ldx_ext(Bus&);
    uint8_t op_ldx_idx(Bus&);
    uint8_t op_ldy_imm(Bus&);
    uint8_t op_ldy_dir(Bus&);
    uint8_t op_ldy_ext(Bus&);
    uint8_t op_ldy_idx(Bus&);
    uint8_t op_ldu_imm(Bus&);
    uint8_t op_ldu_dir(Bus&);
    uint8_t op_ldu_ext(Bus&);
    uint8_t op_ldu_idx(Bus&);
    uint8_t op_lds_imm(Bus&);
    uint8_t op_lds_dir(Bus&);
    uint8_t op_lds_ext(Bus&);
    uint8_t op_lds_idx(Bus&);

    uint8_t op_stx_dir(Bus&);
    uint8_t op_stx_ext(Bus&);
    uint8_t op_stx_idx(Bus&);
    uint8_t op_sty_dir(Bus&);
    uint8_t op_sty_ext(Bus&);
    uint8_t op_sty_idx(Bus&);
    uint8_t op_stu_dir(Bus&);
    uint8_t op_stu_ext(Bus&);
    uint8_t op_stu_idx(Bus&);
    uint8_t op_sts_dir(Bus&);
    uint8_t op_sts_ext(Bus&);
    uint8_t op_sts_idx(Bus&);

    uint8_t op_cmpx_imm(Bus&);
    uint8_t op_cmpx_dir(Bus&);
    uint8_t op_cmpx_ext(Bus&);
    uint8_t op_cmpx_idx(Bus&);
    uint8_t op_cmpy_imm(Bus&);
    uint8_t op_cmpy_dir(Bus&);
    uint8_t op_cmpy_ext(Bus&);
    uint8_t op_cmpy_idx(Bus&);
    uint8_t op_cmpu_imm(Bus&);
    uint8_t op_cmpu_dir(Bus&);
    uint8_t op_cmpu_ext(Bus&);
    uint8_t op_cmpu_idx(Bus&);
    uint8_t op_cmps_imm(Bus&);
    uint8_t op_cmps_dir(Bus&);
    uint8_t op_cmps_ext(Bus&);
    uint8_t op_cmps_idx(Bus&);

    // Misc
    uint8_t op_abx(Bus&);
    uint8_t op_sex(Bus&);
    uint8_t op_andcc(Bus&);
    uint8_t op_orcc(Bus&);
    uint8_t op_daa(Bus&);

    // 6309 extensions
    uint8_t op_ldq_imm(Bus&);
    uint8_t op_ldq_dir(Bus&);
    uint8_t op_ldq_ext(Bus&);
    uint8_t op_ldq_idx(Bus&);
    uint8_t op_stq_dir(Bus&);
    uint8_t op_stq_ext(Bus&);
    uint8_t op_stq_idx(Bus&);

    uint8_t op_ldw_imm(Bus&);
    uint8_t op_ldw_dir(Bus&);
    uint8_t op_ldw_ext(Bus&);
    uint8_t op_ldw_idx(Bus&);
    uint8_t op_stw_dir(Bus&);
    uint8_t op_stw_ext(Bus&);
    uint8_t op_stw_idx(Bus&);

    uint8_t op_addw_imm(Bus&);
    uint8_t op_addw_dir(Bus&);
    uint8_t op_addw_ext(Bus&);
    uint8_t op_addw_idx(Bus&);
    uint8_t op_subw_imm(Bus&);
    uint8_t op_subw_dir(Bus&);
    uint8_t op_subw_ext(Bus&);
    uint8_t op_subw_idx(Bus&);
    uint8_t op_cmpw_imm(Bus&);
    uint8_t op_cmpw_dir(Bus&);
    uint8_t op_cmpw_ext(Bus&);
    uint8_t op_cmpw_idx(Bus&);
    uint8_t op_incw_inh(Bus&);
    uint8_t op_decw_inh(Bus&);
    uint8_t op_tstw_inh(Bus&);
    uint8_t op_clrw_inh(Bus&);

    uint8_t op_adde_imm(Bus&);
    uint8_t op_adde_dir(Bus&);
    uint8_t op_adde_ext(Bus&);
    uint8_t op_adde_idx(Bus&);
    uint8_t op_addf_imm(Bus&);
    uint8_t op_addf_dir(Bus&);
    uint8_t op_addf_ext(Bus&);
    uint8_t op_addf_idx(Bus&);
    uint8_t op_sube_imm(Bus&);
    uint8_t op_sube_dir(Bus&);
    uint8_t op_sube_ext(Bus&);
    uint8_t op_sube_idx(Bus&);
    uint8_t op_subf_imm(Bus&);
    uint8_t op_subf_dir(Bus&);
    uint8_t op_subf_ext(Bus&);
    uint8_t op_subf_idx(Bus&);
    uint8_t op_cmpe_imm(Bus&);
    uint8_t op_cmpe_dir(Bus&);
    uint8_t op_cmpe_ext(Bus&);
    uint8_t op_cmpe_idx(Bus&);
    uint8_t op_cmpf_imm(Bus&);
    uint8_t op_cmpf_dir(Bus&);
    uint8_t op_cmpf_ext(Bus&);
    uint8_t op_cmpf_idx(Bus&);

    uint8_t op_adcd_dir(Bus&);
    uint8_t op_adcd_ext(Bus&);
    uint8_t op_adcd_idx(Bus&);
    uint8_t op_adcd_imm(Bus&);
    uint8_t op_sbcd_dir(Bus&);
    uint8_t op_sbcd_ext(Bus&);
    uint8_t op_sbcd_idx(Bus&);
    uint8_t op_sbcd_imm(Bus&);
    uint8_t op_ord_dir(Bus&);
    uint8_t op_ord_ext(Bus&);
    uint8_t op_ord_idx(Bus&);
    uint8_t op_ord_imm(Bus&);

    uint8_t op_divq_imm(Bus&);
    uint8_t op_divq_dir(Bus&);
    uint8_t op_divq_ext(Bus&);
    uint8_t op_divq_idx(Bus&);
    uint8_t op_muld_imm(Bus&);
    uint8_t op_muld_dir(Bus&);
    uint8_t op_muld_ext(Bus&);
    uint8_t op_muld_idx(Bus&);

    uint8_t op_addr(Bus&);
    uint8_t op_subr(Bus&);
    uint8_t op_cmpr(Bus&);
    uint8_t op_sbcr(Bus&);
    uint8_t op_adcr(Bus&);
    uint8_t op_orr(Bus&);

    uint8_t op_lsl_d(Bus&);
    uint8_t op_rold(Bus&);
    uint8_t op_rord(Bus&);
    uint8_t op_lslw_inh(Bus&);
    uint8_t op_rolw(Bus&);
    uint8_t op_rorw(Bus&);
    uint8_t op_ldmd(Bus&);
    uint8_t op_sexw(Bus&);

    uint8_t op_tfm_pp(Bus&);
    uint8_t op_tfm_mm(Bus&);
    uint8_t op_tfm_pn(Bus&);
    uint8_t op_tfm_np(Bus&);

    uint8_t op_aim_dir(Bus&);
    uint8_t op_aim_idx(Bus&);
    uint8_t op_aim_ext(Bus&);
    uint8_t op_eim_dir(Bus&);
    uint8_t op_eim_idx(Bus&);
    uint8_t op_eim_ext(Bus&);
    uint8_t op_oim_dir(Bus&);
    uint8_t op_oim_idx(Bus&);
    uint8_t op_oim_ext(Bus&);
    uint8_t op_tim_dir(Bus&);
    uint8_t op_tim_idx(Bus&);
    uint8_t op_tim_ext(Bus&);

    // Bit transfer ops (direct only)
    uint8_t op_band(Bus&);
    uint8_t op_biand(Bus&);
    uint8_t op_bor(Bus&);
    uint8_t op_bior(Bus&);
    uint8_t op_beor(Bus&);
    uint8_t op_bieor(Bus&);
    uint8_t op_ldbt(Bus&);
    uint8_t op_stbt(Bus&);

    uint8_t op_invalid(Bus&);

    CpuMode mode_{};
    Registers regs_{};
    uint64_t cycles_executed_{};
    uint16_t last_pc_{};
    uint8_t last_opcode_{};
    uint8_t last_prefix_{};

    Handler page0_[256]{};
    Handler page10_[256]{};
    Handler page11_[256]{};
    bool sync_wait_{false};
};

} // namespace microlind
