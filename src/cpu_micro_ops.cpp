#include "microlind/cpu.hpp"

#include "microlind/bus.hpp"
#include "cpu_detail.hpp"

namespace microlind {

bool Cpu::has_pending_micro_ops() const {
    return micro_op_.kind != MicroOpKind::None;
}

void Cpu::discard_micro_ops() {
    micro_op_ = {};
}

bool Cpu::start_micro_op(Bus& bus, uint8_t opcode) {
    MicroOpKind kind = MicroOpKind::None;
    int32_t total_cycles = 0;
    uint8_t prefix = 0x00;
    uint8_t direct_offset = 0x00;
    uint32_t initial_data32 = 0;
    bool branch_taken = false;
    bool indexed_indirect = false;
    MicroOpTarget target = MicroOpTarget::None;
    MicroOpWidth width = MicroOpWidth::None;
    switch (opcode) {
    case 0x13:
        kind = MicroOpKind::Sync;
        total_cycles = 2;
        break;
    case 0x12:
        kind = MicroOpKind::Nop;
        total_cycles = 2;
        break;
    case 0x19:
    case 0x14:
    case 0x1D:
    case 0x3A:
    case 0x3D:
        kind = MicroOpKind::MiscInherent;
        switch (opcode) {
        case 0x3D:
            total_cycles = 11;
            break;
        case 0x14:
            total_cycles = 4;
            break;
        case 0x3A:
            total_cycles = 3;
            break;
        default:
            total_cycles = 2;
            break;
        }
        break;
    case 0x1A:
    case 0x1C:
        kind = MicroOpKind::CcImmediate;
        total_cycles = 3;
        break;
    case 0x1E:
    case 0x1F:
        kind = MicroOpKind::RegisterTransfer;
        total_cycles = opcode == 0x1E ? 8 : 6;
        break;
    case 0x86:
        kind = MicroOpKind::LoadImmediate;
        target = MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = 2;
        break;
    case 0x96:
        kind = MicroOpKind::LoadDirect;
        target = MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = 4;
        break;
    case 0x97:
        kind = MicroOpKind::StoreDirect;
        target = MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = 4;
        break;
    case 0xB6:
        kind = MicroOpKind::LoadExtended;
        target = MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = 5;
        break;
    case 0xB7:
        kind = MicroOpKind::StoreExtended;
        target = MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = 5;
        break;
    case 0xC6:
        kind = MicroOpKind::LoadImmediate;
        target = MicroOpTarget::B;
        width = MicroOpWidth::Byte;
        total_cycles = 2;
        break;
    case 0xD6:
        kind = MicroOpKind::LoadDirect;
        target = MicroOpTarget::B;
        width = MicroOpWidth::Byte;
        total_cycles = 4;
        break;
    case 0xD7:
        kind = MicroOpKind::StoreDirect;
        target = MicroOpTarget::B;
        width = MicroOpWidth::Byte;
        total_cycles = 4;
        break;
    case 0xF6:
        kind = MicroOpKind::LoadExtended;
        target = MicroOpTarget::B;
        width = MicroOpWidth::Byte;
        total_cycles = 5;
        break;
    case 0xF7:
        kind = MicroOpKind::StoreExtended;
        target = MicroOpTarget::B;
        width = MicroOpWidth::Byte;
        total_cycles = 5;
        break;
    case 0xCC:
        kind = MicroOpKind::LoadImmediate;
        target = MicroOpTarget::D;
        width = MicroOpWidth::Word;
        total_cycles = 3;
        break;
    case 0xCD:
        kind = MicroOpKind::LoadImmediate;
        target = MicroOpTarget::Q;
        width = MicroOpWidth::Long;
        total_cycles = 5;
        break;
    case 0xDC:
        kind = MicroOpKind::LoadDirect;
        target = MicroOpTarget::D;
        width = MicroOpWidth::Word;
        total_cycles = 5;
        break;
    case 0xFC:
        kind = MicroOpKind::LoadExtended;
        target = MicroOpTarget::D;
        width = MicroOpWidth::Word;
        total_cycles = 6;
        break;
    case 0xDD:
        kind = MicroOpKind::StoreDirect;
        target = MicroOpTarget::D;
        width = MicroOpWidth::Word;
        total_cycles = 5;
        break;
    case 0xFD:
        kind = MicroOpKind::StoreExtended;
        target = MicroOpTarget::D;
        width = MicroOpWidth::Word;
        total_cycles = 6;
        break;
    case 0x16:
        kind = MicroOpKind::Lbra;
        total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
        branch_taken = true;
        break;
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x23:
    case 0x24:
    case 0x25:
    case 0x26:
    case 0x27:
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
        kind = MicroOpKind::Branch;
        branch_taken = branch_condition(opcode, regs_);
        total_cycles = branch_taken ? 3 : 2;
        break;
    case 0x8D:
        kind = MicroOpKind::Bsr;
        total_cycles = is_native_hd6309(regs_, mode_) ? 6 : 7;
        break;
    case 0x17:
        kind = MicroOpKind::Lbsr;
        total_cycles = is_native_hd6309(regs_, mode_) ? 7 : 9;
        break;
    case 0x39:
        kind = MicroOpKind::Rts;
        total_cycles = 5;
        break;
    case 0x3B:
        kind = MicroOpKind::Rti;
        total_cycles = (bus.peek8(regs_.s) & CC_E)
            ? (native_hd6309_frame(regs_, mode_) ? 17 : 15)
            : 6;
        break;
    case 0x3C:
        kind = MicroOpKind::Cwai;
        total_cycles = native_hd6309_frame(regs_, mode_) ? 20 : 22;
        break;
    case 0x3F:
        kind = MicroOpKind::Swi;
        total_cycles = native_hd6309_frame(regs_, mode_) ? 21 : 19;
        break;
    case 0x34:
    case 0x36:
        kind = MicroOpKind::StackPush;
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        total_cycles = static_cast<uint8_t>((is_native_hd6309(regs_, mode_) ? 4 : 5) + stack_mask_byte_count(direct_offset));
        break;
    case 0x35:
    case 0x37:
        kind = MicroOpKind::StackPull;
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        total_cycles = static_cast<uint8_t>((is_native_hd6309(regs_, mode_) ? 4 : 5) + stack_mask_byte_count(direct_offset));
        break;
    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
            kind = MicroOpKind::IndexedLea;
            total_cycles = static_cast<uint8_t>(4 + *cycle_add);
            indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
        }
        break;
    case 0x9D:
        kind = MicroOpKind::JsrDirect;
        total_cycles = is_native_hd6309(regs_, mode_) ? 6 : 7;
        break;
    case 0xBD:
        kind = MicroOpKind::JsrExtended;
        total_cycles = is_native_hd6309(regs_, mode_) ? 7 : 8;
        break;
    case 0xAD:
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
            kind = MicroOpKind::JsrIndexed;
            total_cycles = static_cast<uint8_t>((is_native_hd6309(regs_, mode_) ? 6 : 7) + *cycle_add);
            indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
        }
        break;
    case 0x0E:
        kind = MicroOpKind::JmpDirect;
        total_cycles = is_native_hd6309(regs_, mode_) ? 2 : 3;
        break;
    case 0x7E:
        kind = MicroOpKind::JmpExtended;
        total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
        break;
    case 0x6E:
        direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
        if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
            kind = MicroOpKind::JmpIndexed;
            total_cycles = static_cast<uint8_t>(3 + *cycle_add);
            indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
        }
        break;
    case 0x40:
    case 0x50:
    case 0x43:
    case 0x53:
    case 0x44:
    case 0x54:
    case 0x46:
    case 0x56:
    case 0x47:
    case 0x57:
    case 0x48:
    case 0x58:
    case 0x49:
    case 0x59:
    case 0x4F:
    case 0x5F:
    case 0x4D:
    case 0x5D:
    case 0x4A:
    case 0x5A:
    case 0x4C:
    case 0x5C:
        kind = MicroOpKind::RegisterUnary;
        target = (opcode & 0x10) != 0 ? MicroOpTarget::B : MicroOpTarget::A;
        width = MicroOpWidth::Byte;
        total_cycles = is_native_hd6309(regs_, mode_) ? 1 : 2;
        break;
    case 0x8C:
        kind = MicroOpKind::Cmp16Immediate;
        total_cycles = cmp16_immediate_cycles(prefix, regs_, mode_);
        break;
    case 0x9C:
        kind = MicroOpKind::Cmp16Direct;
        total_cycles = cmp16_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBC:
        kind = MicroOpKind::Cmp16Extended;
        total_cycles = cmp16_extended_cycles(prefix, regs_, mode_);
        break;
    case 0x8E:
    case 0xCE:
        kind = MicroOpKind::WordLoadImmediate;
        total_cycles = word_load_immediate_cycles(prefix, opcode, regs_, mode_);
        break;
    case 0x9E:
    case 0xDE:
        kind = MicroOpKind::WordLoadDirect;
        total_cycles = word_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBE:
    case 0xFE:
        kind = MicroOpKind::WordLoadExtended;
        total_cycles = word_extended_cycles(prefix, regs_, mode_);
        break;
    case 0x9F:
    case 0xDF:
        kind = MicroOpKind::WordStoreDirect;
        total_cycles = word_direct_cycles(prefix, regs_, mode_);
        break;
    case 0xBF:
    case 0xFF:
        kind = MicroOpKind::WordStoreExtended;
        total_cycles = word_extended_cycles(prefix, regs_, mode_);
        break;
    default:
        if (is_word_load_indexed_opcode(prefix, opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedWordLoad;
                total_cycles = static_cast<uint8_t>(word_indexed_base_cycles(prefix) + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (is_word_store_indexed_opcode(prefix, opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedWordStore;
                total_cycles = static_cast<uint8_t>(word_indexed_base_cycles(prefix) + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (is_indexed_data_opcode(opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                switch (opcode) {
                case 0xA6:
                case 0xE6:
                    kind = MicroOpKind::IndexedLoad8;
                    total_cycles = static_cast<uint8_t>(4 + *cycle_add);
                    break;
                case 0xEC:
                    kind = MicroOpKind::IndexedLoad16;
                    total_cycles = static_cast<uint8_t>(5 + *cycle_add);
                    break;
                case 0xA7:
                case 0xE7:
                    kind = MicroOpKind::IndexedStore8;
                    total_cycles = static_cast<uint8_t>(4 + *cycle_add);
                    break;
                case 0xED:
                    kind = MicroOpKind::IndexedStore16;
                    total_cycles = static_cast<uint8_t>(5 + *cycle_add);
                    break;
                default:
                    break;
                }
                break;
            }
        }
        if (is_alu8_indexed_opcode(opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedAlu8;
                total_cycles = static_cast<uint8_t>(4 + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (is_alu16_indexed_opcode(opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedAlu16;
                total_cycles = static_cast<uint8_t>(6 + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (is_cmp16_indexed_opcode(0x00, opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedCmp16;
                total_cycles = static_cast<uint8_t>(6 + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (opcode == 0x10) {
            const uint8_t next = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (next >= 0x21 && next <= 0x2F) {
                kind = MicroOpKind::LongBranch;
                branch_taken = branch_condition(next, regs_);
                total_cycles = branch_taken ? 6 : 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x3F) {
                kind = MicroOpKind::Swi;
                total_cycles = native_hd6309_frame(regs_, mode_) ? 22 : 20;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next >= 0x38 && next <= 0x3B) {
                kind = (next == 0x38 || next == 0x3A) ? MicroOpKind::WStackPush : MicroOpKind::WStackPull;
                total_cycles = 6;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next >= 0x30 && next <= 0x37) {
                kind = MicroOpKind::RegisterAlu;
                total_cycles = 4;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_d_or_w_unary_opcode(next)) {
                kind = MicroOpKind::PrefixedRegisterUnary;
                total_cycles = 3;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x83) {
                kind = MicroOpKind::CmpdImmediate;
                total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x93) {
                kind = MicroOpKind::CmpdDirect;
                total_cycles = is_native_hd6309(regs_, mode_) ? 5 : 7;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0xB3) {
                kind = MicroOpKind::CmpdExtended;
                total_cycles = is_native_hd6309(regs_, mode_) ? 6 : 8;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_d_alu_immediate_opcode(next)) {
                kind = MicroOpKind::DAluImmediate;
                total_cycles = 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_d_alu_direct_opcode(next)) {
                kind = MicroOpKind::DAluDirect;
                total_cycles = 7;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_d_alu_extended_opcode(next)) {
                kind = MicroOpKind::DAluExtended;
                total_cycles = 8;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_d_alu_indexed_opcode(next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedDAlu;
                    total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_cmp16_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Immediate;
                total_cycles = cmp16_immediate_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_direct_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Direct;
                total_cycles = cmp16_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_extended_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Extended;
                total_cycles = cmp16_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0xDC || next == 0xFC || next == 0xDD || next == 0xFD) {
                switch (next) {
                case 0xDC:
                    kind = MicroOpKind::QLoadDirect;
                    total_cycles = 8;
                    break;
                case 0xFC:
                    kind = MicroOpKind::QLoadExtended;
                    total_cycles = 9;
                    break;
                case 0xDD:
                    kind = MicroOpKind::QStoreDirect;
                    total_cycles = 8;
                    break;
                case 0xFD:
                    kind = MicroOpKind::QStoreExtended;
                    total_cycles = 9;
                    break;
                default:
                    break;
                }
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0x86 || next == 0x96 || next == 0xB6 ||
                next == 0x97 || next == 0xB7 ||
                next == 0x80 || next == 0x8B || next == 0x90 || next == 0x9B || next == 0xB0 || next == 0xBB ||
                next == 0x81 || next == 0x91 || next == 0xB1) {
                switch (next) {
                case 0x86:
                    kind = MicroOpKind::WLoadImmediate;
                    total_cycles = 4;
                    break;
                case 0x96:
                    kind = MicroOpKind::WLoadDirect;
                    total_cycles = 6;
                    break;
                case 0xB6:
                    kind = MicroOpKind::WLoadExtended;
                    total_cycles = 7;
                    break;
                case 0x97:
                    kind = MicroOpKind::WStoreDirect;
                    total_cycles = 6;
                    break;
                case 0xB7:
                    kind = MicroOpKind::WStoreExtended;
                    total_cycles = 7;
                    break;
                case 0x80:
                case 0x8B:
                    kind = MicroOpKind::WAluImmediate;
                    total_cycles = 5;
                    break;
                case 0x90:
                case 0x9B:
                    kind = MicroOpKind::WAluDirect;
                    total_cycles = 7;
                    break;
                case 0xB0:
                case 0xBB:
                    kind = MicroOpKind::WAluExtended;
                    total_cycles = 8;
                    break;
                case 0x81:
                    kind = MicroOpKind::WCmpImmediate;
                    total_cycles = 5;
                    break;
                case 0x91:
                    kind = MicroOpKind::WCmpDirect;
                    total_cycles = 7;
                    break;
                case 0xB1:
                    kind = MicroOpKind::WCmpExtended;
                    total_cycles = 8;
                    break;
                default:
                    break;
                }
                prefix = opcode;
                opcode = next;
                break;
            }
            if (next == 0xA3) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedCmpd;
                    total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (next == 0xA0 || next == 0xA1 || next == 0xAB) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = next == 0xA1 ? MicroOpKind::IndexedWCmp : MicroOpKind::IndexedWAlu;
                    total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (next == 0xA6 || next == 0xA7 || next == 0xEC || next == 0xED) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    switch (next) {
                    case 0xA6:
                        kind = MicroOpKind::IndexedWLoad;
                        total_cycles = static_cast<uint8_t>(6 + *cycle_add);
                        break;
                    case 0xA7:
                        kind = MicroOpKind::IndexedWStore;
                        total_cycles = static_cast<uint8_t>(6 + *cycle_add);
                        break;
                    case 0xEC:
                        kind = MicroOpKind::IndexedQLoad;
                        total_cycles = static_cast<uint8_t>(8 + *cycle_add);
                        break;
                    case 0xED:
                        kind = MicroOpKind::IndexedQStore;
                        total_cycles = static_cast<uint8_t>(8 + *cycle_add);
                        break;
                    default:
                        break;
                    }
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_cmp16_indexed_opcode(opcode, next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedCmp16;
                    total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_word_load_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadImmediate;
                total_cycles = word_load_immediate_cycles(opcode, next, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_direct_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadDirect;
                total_cycles = word_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_extended_opcode(opcode, next)) {
                kind = MicroOpKind::WordLoadExtended;
                total_cycles = word_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_store_direct_opcode(opcode, next)) {
                kind = MicroOpKind::WordStoreDirect;
                total_cycles = word_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_store_extended_opcode(opcode, next)) {
                kind = MicroOpKind::WordStoreExtended;
                total_cycles = word_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_word_load_indexed_opcode(opcode, next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedWordLoad;
                    total_cycles = static_cast<uint8_t>(word_indexed_base_cycles(opcode) + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_word_store_indexed_opcode(opcode, next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedWordStore;
                    total_cycles = static_cast<uint8_t>(word_indexed_base_cycles(opcode) + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
        }
        if (opcode == 0x11) {
            const uint8_t next = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (next == 0x3F) {
                kind = MicroOpKind::Swi;
                total_cycles = native_hd6309_frame(regs_, mode_) ? 22 : 20;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_divd_opcode(next)) {
                switch (next) {
                case 0x8D:
                    kind = MicroOpKind::DivDImmediate;
                    total_cycles = 25;
                    break;
                case 0x9D:
                    kind = MicroOpKind::DivDDirect;
                    total_cycles = 27;
                    break;
                case 0xBD:
                    kind = MicroOpKind::DivDExtended;
                    total_cycles = 28;
                    break;
                case 0xAD:
                    direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                    if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                        kind = MicroOpKind::IndexedDivD;
                        total_cycles = 27 + *cycle_add;
                        indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    }
                    break;
                default:
                    break;
                }
                if (kind != MicroOpKind::None) {
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_divq_opcode(next)) {
                switch (next) {
                case 0x8E:
                    kind = MicroOpKind::DivQImmediate;
                    total_cycles = 36;
                    break;
                case 0x9E:
                    kind = MicroOpKind::DivQDirect;
                    total_cycles = 36;
                    break;
                case 0xBE:
                    kind = MicroOpKind::DivQExtended;
                    total_cycles = 37;
                    break;
                case 0xAE:
                    direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                    if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                        kind = MicroOpKind::IndexedDivQ;
                        total_cycles = 36 + *cycle_add;
                        indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    }
                    break;
                default:
                    break;
                }
                if (kind != MicroOpKind::None) {
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_bit_transfer_opcode(next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (bit_transfer_postbyte_is_valid(direct_offset)) {
                    kind = MicroOpKind::BitTransfer;
                    total_cycles = next == 0x37 ? 8 : 7;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_tfm_opcode(next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (tfm_postbyte_is_valid(direct_offset)) {
                    initial_data32 = tfm_transfer_count(regs_);
                    kind = MicroOpKind::Tfm;
                    total_cycles = static_cast<int32_t>(6u + (3u * initial_data32));
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (next == 0x3C || next == 0x3D) {
                kind = MicroOpKind::MdImmediate;
                total_cycles = next == 0x3C ? 4 : 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_hd6309_e_or_f_unary_opcode(next)) {
                kind = MicroOpKind::PrefixedRegisterUnary;
                total_cycles = 3;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_ef_alu_immediate_opcode(next)) {
                kind = MicroOpKind::EFAluImmediate;
                total_cycles = 3;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_ef_alu_direct_opcode(next)) {
                kind = MicroOpKind::EFAluDirect;
                total_cycles = 5;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_ef_alu_extended_opcode(next)) {
                kind = MicroOpKind::EFAluExtended;
                total_cycles = 6;
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_ef_alu_indexed_opcode(next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedEFAlu;
                    total_cycles = static_cast<uint8_t>(5 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
            if (is_cmp16_immediate_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Immediate;
                total_cycles = cmp16_immediate_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_direct_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Direct;
                total_cycles = cmp16_direct_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_extended_opcode(opcode, next)) {
                kind = MicroOpKind::Cmp16Extended;
                total_cycles = cmp16_extended_cycles(opcode, regs_, mode_);
                prefix = opcode;
                opcode = next;
                break;
            }
            if (is_cmp16_indexed_opcode(opcode, next)) {
                direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
                if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                    kind = MicroOpKind::IndexedCmp16;
                    total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                    indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                    prefix = opcode;
                    opcode = next;
                    break;
                }
            }
        }
        if (is_alu8_immediate_opcode(opcode)) {
            kind = MicroOpKind::Alu8Immediate;
            total_cycles = 2;
            break;
        }
        if (is_alu8_direct_opcode(opcode)) {
            kind = MicroOpKind::Alu8Direct;
            total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
            break;
        }
        if (is_alu8_extended_opcode(opcode)) {
            kind = MicroOpKind::Alu8Extended;
            total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 5;
            break;
        }
        if (is_alu16_immediate_opcode(opcode)) {
            kind = MicroOpKind::Alu16Immediate;
            total_cycles = is_native_hd6309(regs_, mode_) ? 3 : 4;
            break;
        }
        if (is_alu16_direct_opcode(opcode)) {
            kind = MicroOpKind::Alu16Direct;
            total_cycles = is_native_hd6309(regs_, mode_) ? 4 : 6;
            break;
        }
        if (is_alu16_extended_opcode(opcode)) {
            kind = MicroOpKind::Alu16Extended;
            total_cycles = is_native_hd6309(regs_, mode_) ? 5 : 7;
            break;
        }
        if (is_immediate_memory_direct_opcode(opcode)) {
            kind = MicroOpKind::ImmediateMemoryDirect;
            total_cycles = 6;
            break;
        }
        if (is_immediate_memory_extended_opcode(opcode)) {
            kind = MicroOpKind::ImmediateMemoryExtended;
            total_cycles = 7;
            break;
        }
        if (is_immediate_memory_indexed_opcode(opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 2));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedImmediateMemory;
                total_cycles = static_cast<uint8_t>(7 + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        if (is_memory_unary_direct_opcode(opcode)) {
            kind = MicroOpKind::MemoryUnaryDirect;
            total_cycles = 6;
            break;
        }
        if (is_memory_unary_extended_opcode(opcode)) {
            kind = MicroOpKind::MemoryUnaryExtended;
            total_cycles = 7;
            break;
        }
        if (is_memory_unary_indexed_opcode(opcode)) {
            direct_offset = bus.peek8(static_cast<uint16_t>(regs_.pc + 1));
            if (const auto cycle_add = indexed_cycle_add_for_micro_op(direct_offset, regs_, mode_)) {
                kind = MicroOpKind::IndexedMemoryUnary;
                total_cycles = static_cast<uint8_t>(6 + *cycle_add);
                indexed_indirect = (direct_offset & 0x80) != 0 && (direct_offset & 0x10) != 0;
                break;
            }
        }
        return false;
    }

    const Instruction& inst = prefix == 0x10
        ? instructions10_[opcode]
        : prefix == 0x11
            ? instructions11_[opcode]
            : instructions0_[opcode];
    if (mode_ == CpuMode::MC6809 && inst.hd6309_only) {
        return false;
    }

    last_pc_ = regs_.pc;
    last_prefix_ = prefix;
    last_opcode_ = opcode;
    micro_op_ = MicroOpState{
        kind,
        regs_.pc,
        opcode,
        prefix,
        0,
        total_cycles,
        direct_offset,
        0,
        0,
        initial_data32,
        branch_taken,
        indexed_indirect,
        target,
        width,
    };
    return true;
}

bool Cpu::start_interrupt_micro_op(uint8_t interrupt_source, bool stack_frame) {
    const auto source = static_cast<InterruptSource>(interrupt_source);
    last_pc_ = regs_.pc;
    last_prefix_ = 0x00;
    last_opcode_ = 0x00;
    if (stack_frame && interrupt_uses_full_frame(regs_, source)) {
        regs_.cc = static_cast<uint8_t>(regs_.cc | CC_E);
    } else if (stack_frame) {
        regs_.cc = static_cast<uint8_t>(regs_.cc & ~CC_E);
    }
    micro_op_ = MicroOpState{
        stack_frame ? MicroOpKind::IrqEntry : MicroOpKind::InterruptVector,
        regs_.pc,
        interrupt_source,
        0x00,
        0,
        stack_frame ? interrupt_total_cycles(regs_, mode_, source) : 2,
        0,
        0,
        0,
        0,
        false,
        false,
        MicroOpTarget::None,
        MicroOpWidth::None,
    };
    return true;
}

CpuMicrocycleStatus Cpu::micro_op_status(bool instruction_started, bool instruction_complete) const {
    const std::size_t completed_cycles = instruction_complete
        ? static_cast<std::size_t>(micro_op_.total_cycles)
        : static_cast<std::size_t>(micro_op_.step);
    const std::size_t total_cycles = static_cast<std::size_t>(micro_op_.total_cycles);
    const std::size_t pending = total_cycles > completed_cycles
        ? total_cycles - completed_cycles
        : 0u;
    return CpuMicrocycleStatus{
        instruction_started,
        instruction_complete,
        CpuTickResult{static_cast<uint32_t>(micro_op_.total_cycles)},
        pending,
    };
}

BusSignals Cpu::micro_op_signals() const {
    switch (micro_op_.kind) {
    case MicroOpKind::Nop:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::MiscInherent:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::CcImmediate:
    case MicroOpKind::RegisterTransfer:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::PrefixedRegisterUnary:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        return internal_cycle(regs_.pc);
    case MicroOpKind::EFAluImmediate:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::MdImmediate:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::RegisterAlu:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::BitTransfer:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (bit_transfer_writes_memory(micro_op_.opcode) && micro_op_.step == 5) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivDImmediate:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 3) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 4) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivQImmediate:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 4) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 5) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivDDirect:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 4) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 5) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivQDirect:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 5) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 6) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivDExtended:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 5) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 6) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::DivQExtended:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == 5) return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        if (micro_op_.branch_taken && micro_op_.step == 6) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == 7) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Tfm: {
        const int32_t transfer_bus_steps = static_cast<int32_t>(2u * micro_op_.data32);
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 3 && micro_op_.step < 3 + transfer_bus_steps) {
            const bool read_step = ((micro_op_.step - 3) % 2) == 0;
            const uint8_t src_code = static_cast<uint8_t>(micro_op_.direct_offset >> 4);
            const uint8_t dst_code = static_cast<uint8_t>(micro_op_.direct_offset & 0x0F);
            if (read_step) {
                if (const uint16_t* source = tfm_register_pointer(regs_, src_code)) {
                    return read_cycle(*source, BusCycleKind::OperandRead);
                }
            } else if (const uint16_t* destination = tfm_register_pointer(regs_, dst_code)) {
                return write_cycle(*destination, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
            }
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Alu16Immediate:
    case MicroOpKind::DAluImmediate:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.kind == MicroOpKind::DAluImmediate && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if ((micro_op_.kind == MicroOpKind::DAluImmediate && (micro_op_.step == 2 || micro_op_.step == 3)) ||
            (micro_op_.kind == MicroOpKind::Alu16Immediate && (micro_op_.step == 1 || micro_op_.step == 2))) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::CmpdImmediate:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2 || micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Cmp16Immediate: {
        const uint8_t operand_high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t operand_low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_high_step || micro_op_.step == operand_low_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadImmediate:
    case MicroOpKind::WLoadImmediate:
    case MicroOpKind::WAluImmediate:
    case MicroOpKind::WCmpImmediate: {
        const uint8_t operand_high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t operand_low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_high_step || micro_op_.step == operand_low_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::LoadImmediate:
    case MicroOpKind::Alu8Immediate:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : read_cycle(regs_.pc, BusCycleKind::OperandRead);
    case MicroOpKind::LoadDirect:
    case MicroOpKind::Alu8Direct:
    case MicroOpKind::Alu16Direct:
    case MicroOpKind::CmpdDirect:
    case MicroOpKind::DAluDirect: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        const bool prefixed_d_op =
            micro_op_.kind == MicroOpKind::CmpdDirect || micro_op_.kind == MicroOpKind::DAluDirect;
        if (prefixed_d_op && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        const uint8_t operand_step = prefixed_d_op ? 2 : 1;
        const uint8_t high_step = prefixed_d_op ? 3 : 2;
        const uint8_t low_step = prefixed_d_op ? 4 : 3;
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (((micro_op_.kind == MicroOpKind::LoadDirect && micro_op_.width == MicroOpWidth::Word) ||
             micro_op_.kind == MicroOpKind::Alu16Direct ||
             prefixed_d_op) &&
            micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Cmp16Direct:
    case MicroOpKind::WLoadDirect:
    case MicroOpKind::WAluDirect:
    case MicroOpKind::WCmpDirect: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WStoreDirect: {
        const uint8_t operand_step = 2;
        const uint8_t high_step = 3;
        const uint8_t low_step = 4;
        const uint16_t value = reg_w();
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::EFAluDirect:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::QLoadDirect:
    case MicroOpKind::QStoreDirect: {
        const uint8_t operand_step = 2;
        const uint8_t data_step = 3;
        const uint32_t value = reg_q();
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= data_step && micro_op_.step < data_step + 4) {
            const uint16_t address = static_cast<uint16_t>(micro_op_.effective_address + (micro_op_.step - data_step));
            if (micro_op_.kind == MicroOpKind::QLoadDirect) {
                return read_cycle(address, BusCycleKind::OperandRead);
            }
            const uint8_t shift = static_cast<uint8_t>(24 - 8 * (micro_op_.step - data_step));
            return write_cycle(address, static_cast<uint8_t>((value >> shift) & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadDirect: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordStoreDirect: {
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint16_t value = word_register_value(regs_, micro_op_.prefix, micro_op_.opcode);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == operand_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::LoadExtended:
    case MicroOpKind::Alu8Extended:
    case MicroOpKind::Alu16Extended:
    case MicroOpKind::CmpdExtended:
    case MicroOpKind::DAluExtended: {
        const bool prefixed_d_op =
            micro_op_.kind == MicroOpKind::CmpdExtended || micro_op_.kind == MicroOpKind::DAluExtended;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (prefixed_d_op && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        const uint8_t high_address_step = prefixed_d_op ? 2 : 1;
        const uint8_t low_address_step = prefixed_d_op ? 3 : 2;
        const uint8_t high_data_step = prefixed_d_op ? 4 : 3;
        const uint8_t low_data_step = prefixed_d_op ? 5 : 4;
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (((micro_op_.kind == MicroOpKind::LoadExtended && micro_op_.width == MicroOpWidth::Word) ||
             micro_op_.kind == MicroOpKind::Alu16Extended ||
             prefixed_d_op) &&
            micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Cmp16Extended:
    case MicroOpKind::WLoadExtended:
    case MicroOpKind::WAluExtended:
    case MicroOpKind::WCmpExtended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WStoreExtended: {
        const uint8_t high_address_step = 2;
        const uint8_t low_address_step = 3;
        const uint8_t high_data_step = 4;
        const uint8_t low_data_step = 5;
        const uint16_t value = reg_w();
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_data_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::EFAluExtended:
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::QLoadExtended:
    case MicroOpKind::QStoreExtended: {
        const uint8_t high_address_step = 2;
        const uint8_t low_address_step = 3;
        const uint8_t data_step = 4;
        const uint32_t value = reg_q();
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= data_step && micro_op_.step < data_step + 4) {
            const uint16_t address = static_cast<uint16_t>(micro_op_.effective_address + (micro_op_.step - data_step));
            if (micro_op_.kind == MicroOpKind::QLoadExtended) {
                return read_cycle(address, BusCycleKind::OperandRead);
            }
            const uint8_t shift = static_cast<uint8_t>(24 - 8 * (micro_op_.step - data_step));
            return write_cycle(address, static_cast<uint8_t>((value >> shift) & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordLoadExtended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == low_data_step) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WordStoreExtended: {
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        const uint16_t value = word_register_value(regs_, micro_op_.prefix, micro_op_.opcode);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == high_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == low_address_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == high_data_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == low_data_step) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::ImmediateMemoryDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (immediate_memory_writes(micro_op_.opcode) && micro_op_.step == 4) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::ImmediateMemoryExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 4) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (immediate_memory_writes(micro_op_.opcode) && micro_op_.step == 5) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::IndexedImmediateMemory: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(3 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 3 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (immediate_memory_writes(micro_op_.opcode) && micro_op_.step == operand_step + 1) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::MemoryUnaryDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) == 0x0F && micro_op_.step == 2) {
            return write_cycle(micro_op_.effective_address, 0x00, BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == 2) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) != 0x0D && (micro_op_.opcode & 0x0F) != 0x0F && micro_op_.step == 3) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::MemoryUnaryExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) == 0x0F && micro_op_.step == 3) {
            return write_cycle(micro_op_.effective_address, 0x00, BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == 3) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if ((micro_op_.opcode & 0x0F) != 0x0D && (micro_op_.opcode & 0x0F) != 0x0F && micro_op_.step == 4) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::IndexedMemoryUnary: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint8_t low_opcode = static_cast<uint8_t>(micro_op_.opcode & 0x0F);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (low_opcode == 0x0F && micro_op_.step == operand_step) {
            return write_cycle(micro_op_.effective_address, 0x00, BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == operand_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (low_opcode != 0x0D && low_opcode != 0x0F && micro_op_.step == operand_step + 1) {
            return write_cycle(micro_op_.effective_address, static_cast<uint8_t>(micro_op_.data & 0xFF), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedLoad8:
    case MicroOpKind::IndexedAlu8:
    case MicroOpKind::IndexedLoad16: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.kind == MicroOpKind::IndexedLoad16 && micro_op_.step == operand_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedAlu16:
    case MicroOpKind::IndexedCmpd:
    case MicroOpKind::IndexedDAlu:
    case MicroOpKind::IndexedCmp16:
    case MicroOpKind::IndexedWordLoad:
    case MicroOpKind::IndexedWLoad:
    case MicroOpKind::IndexedWAlu:
    case MicroOpKind::IndexedWCmp: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.step == operand_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedEFAlu: {
        const uint8_t post_step = 2;
        const uint8_t extension_start_step = 3;
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedDivD:
    case MicroOpKind::IndexedDivQ: {
        const uint8_t post_step = 2;
        const uint8_t extension_start_step = 3;
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint8_t vector_step = static_cast<uint8_t>(operand_step + (micro_op_.kind == MicroOpKind::IndexedDivQ ? 2 : 1));
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        if (micro_op_.kind == MicroOpKind::IndexedDivQ && micro_op_.step == operand_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.branch_taken && micro_op_.step == vector_step) return read_cycle(0xFFF0, BusCycleKind::VectorRead);
        if (micro_op_.branch_taken && micro_op_.step == vector_step + 1) return read_cycle(0xFFF1, BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedQLoad: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step >= operand_step && micro_op_.step < operand_step + 4) {
            const uint16_t address = static_cast<uint16_t>(micro_op_.effective_address + (micro_op_.step - operand_step));
            return read_cycle(address, BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedWordStore: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint16_t value = word_register_value(regs_, micro_op_.prefix, micro_op_.opcode);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == operand_step + 1) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedWStore: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint16_t value = reg_w();
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) {
            return write_cycle(micro_op_.effective_address, hi(value), BusCycleKind::OperandWrite);
        }
        if (micro_op_.step == operand_step + 1) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedQStore: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint32_t value = reg_q();
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.prefix != 0x00 && micro_op_.step == 1) {
            return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step == post_step) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= extension_start_step && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step >= operand_step && micro_op_.step < operand_step + 4) {
            const uint8_t shift = static_cast<uint8_t>(24 - 8 * (micro_op_.step - operand_step));
            const uint8_t byte = static_cast<uint8_t>((value >> shift) & 0xFF);
            const uint16_t address = static_cast<uint16_t>(micro_op_.effective_address + (micro_op_.step - operand_step));
            return write_cycle(address, byte, BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedStore8:
    case MicroOpKind::IndexedStore16: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint16_t value = reg_d_value(regs_);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == operand_step) {
            const uint8_t byte = micro_op_.opcode == 0xE7 ? regs_.b : regs_.a;
            return write_cycle(micro_op_.effective_address, byte, BusCycleKind::OperandWrite);
        }
        if (micro_op_.kind == MicroOpKind::IndexedStore16 && micro_op_.step == operand_step + 1) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), lo(value), BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::StoreDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            const uint8_t value = micro_op_.target == MicroOpTarget::B ? regs_.b : regs_.a;
            return write_cycle(micro_op_.effective_address, value, BusCycleKind::OperandWrite);
        }
        if (micro_op_.width == MicroOpWidth::Word && micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), regs_.b, BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::StoreExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            const uint8_t value = micro_op_.target == MicroOpTarget::B ? regs_.b : regs_.a;
            return write_cycle(micro_op_.effective_address, value, BusCycleKind::OperandWrite);
        }
        if (micro_op_.width == MicroOpWidth::Word && micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), regs_.b, BusCycleKind::OperandWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Sync:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::InterruptVector: {
        const auto source = static_cast<InterruptSource>(micro_op_.opcode);
        const uint16_t vector = interrupt_vector(source);
        if (micro_op_.step == 0) return read_cycle(vector, BusCycleKind::VectorRead);
        if (micro_op_.step == 1) return read_cycle(static_cast<uint16_t>(vector + 1), BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IrqEntry:
    case MicroOpKind::Swi: {
        const bool external_interrupt = micro_op_.kind == MicroOpKind::IrqEntry;
        const auto source = static_cast<InterruptSource>(micro_op_.opcode);
        const int32_t stack_start = external_interrupt ? 0 : (micro_op_.prefix == 0x00 ? 1 : 2);
        const int32_t stack_bytes = external_interrupt
            ? interrupt_stack_byte_count(regs_, mode_, source)
            : (native_hd6309_frame(regs_, mode_) ? 14 : 12);
        const int32_t vector_step = stack_start + stack_bytes;
        const uint16_t vector = external_interrupt
            ? interrupt_vector(source)
            : micro_op_.prefix == 0x10
                ? 0xFFF4
                : micro_op_.prefix == 0x11
                    ? 0xFFF2
                    : 0xFFFA;
        if (micro_op_.kind == MicroOpKind::Swi) {
            if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
            if (micro_op_.prefix != 0x00 && micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        }
        if (micro_op_.step >= stack_start && micro_op_.step < vector_step) {
            const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - stack_start);
            return write_cycle(
                static_cast<uint16_t>(regs_.s - 1),
                external_interrupt
                    ? interrupt_stack_push_byte(regs_, mode_, source, byte_index)
                    : interrupt_stack_push_byte(regs_, mode_, InterruptSource::Irq, byte_index),
                BusCycleKind::StackWrite);
        }
        if (micro_op_.step == vector_step) return read_cycle(vector, BusCycleKind::VectorRead);
        if (micro_op_.step == vector_step + 1) return read_cycle(static_cast<uint16_t>(vector + 1), BusCycleKind::VectorRead);
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Cwai: {
        constexpr int32_t stack_start = 2;
        const int32_t stack_end = stack_start + (native_hd6309_frame(regs_, mode_) ? 14 : 12);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= stack_start && micro_op_.step < stack_end) {
            const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - stack_start);
            return write_cycle(
                static_cast<uint16_t>(regs_.s - 1),
                interrupt_stack_push_byte(regs_, mode_, InterruptSource::Irq, byte_index),
                BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Rti: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.s, BusCycleKind::StackRead);
        if (micro_op_.total_cycles == 6) {
            if (micro_op_.step == 2) return read_cycle(regs_.s, BusCycleKind::StackRead);
            if (micro_op_.step == 3) return read_cycle(regs_.s, BusCycleKind::StackRead);
            return internal_cycle(regs_.pc);
        }
        const int32_t last_stack_step = native_hd6309_frame(regs_, mode_) ? 14 : 12;
        if (micro_op_.step >= 2 && micro_op_.step <= last_stack_step) {
            return read_cycle(regs_.s, BusCycleKind::StackRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::Branch:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Lbra:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::LongBranch:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::Bsr:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Lbsr:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::Rts:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.s, BusCycleKind::StackRead);
        if (micro_op_.step == 2) return read_cycle(regs_.s, BusCycleKind::StackRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::StackPush: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - 2);
        if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
            const bool use_u_stack = micro_op_.opcode == 0x36;
            const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
            return write_cycle(
                static_cast<uint16_t>(pointer - 1),
                stack_push_byte(regs_, micro_op_.direct_offset, byte_index, use_u_stack),
                BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::StackPull: {
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        const uint8_t byte_index = static_cast<uint8_t>(micro_op_.step - 2);
        if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
            const bool use_u_stack = micro_op_.opcode == 0x37;
            const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
            return read_cycle(pointer, BusCycleKind::StackRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WStackPush: {
        const bool use_u_stack = micro_op_.opcode == 0x3A;
        const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
        const uint16_t value = reg_w();
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) {
            const uint16_t address = use_u_stack ? static_cast<uint16_t>(pointer - 2) : static_cast<uint16_t>(pointer - 1);
            const uint8_t byte = use_u_stack ? hi(value) : lo(value);
            return write_cycle(address, byte, BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 3) {
            const uint16_t address = use_u_stack ? static_cast<uint16_t>(pointer + 1) : static_cast<uint16_t>(pointer - 1);
            const uint8_t byte = use_u_stack ? lo(value) : hi(value);
            return write_cycle(address, byte, BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::WStackPull: {
        const bool use_u_stack = micro_op_.opcode == 0x3B;
        const uint16_t pointer = use_u_stack ? regs_.u : regs_.s;
        if (micro_op_.step == 0 || micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 2) return read_cycle(pointer, BusCycleKind::StackRead);
        if (micro_op_.step == 3) return read_cycle(static_cast<uint16_t>(pointer + 1), BusCycleKind::StackRead);
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::JsrDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::JsrExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 3) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(micro_op_.data), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == 4) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(micro_op_.data), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    case MicroOpKind::JsrIndexed: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t stack_low_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        if (micro_op_.step == stack_low_step) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), lo(regs_.pc), BusCycleKind::StackWrite);
        }
        if (micro_op_.step == stack_low_step + 1) {
            return write_cycle(static_cast<uint16_t>(regs_.s - 1), hi(regs_.pc), BusCycleKind::StackWrite);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::JmpDirect:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::JmpExtended:
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step == 2) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        return internal_cycle(regs_.pc);
    case MicroOpKind::JmpIndexed: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::IndexedLea: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        if (micro_op_.step == 0) return read_cycle(regs_.pc, BusCycleKind::OpcodeFetch);
        if (micro_op_.step == 1) return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        if (micro_op_.step >= 2 && micro_op_.step < pointer_step) {
            return read_cycle(regs_.pc, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step) {
            return read_cycle(micro_op_.effective_address, BusCycleKind::OperandRead);
        }
        if (micro_op_.indexed_indirect && micro_op_.step == pointer_step + 1) {
            return read_cycle(static_cast<uint16_t>(micro_op_.effective_address + 1), BusCycleKind::OperandRead);
        }
        return internal_cycle(regs_.pc);
    }
    case MicroOpKind::RegisterUnary:
        return micro_op_.step == 0
            ? read_cycle(regs_.pc, BusCycleKind::OpcodeFetch)
            : internal_cycle(regs_.pc);
    case MicroOpKind::None:
        break;
    }
    return {};
}

bool Cpu::prepare_microcycle(Bus& bus, BusSignals& signals, CpuMicrocycleStatus& status) {
    bool instruction_started = false;
    if (!has_pending_micro_ops()) {
        if (sync_wait_ && !interrupt_line_asserted()) {
            return false;
        }
        if (sync_wait_) {
            sync_wait_ = false;
        }

        if (cwai_wait_) {
            if (nmi_pending()) {
                start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Nmi), false);
            } else if (firq_pending()) {
                start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Firq), false);
            } else if (irq_pending()) {
                start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Irq), false);
            } else {
                return false;
            }
            instruction_started = true;
        } else if (nmi_pending()) {
            start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Nmi), true);
            instruction_started = true;
        } else if (firq_pending()) {
            start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Firq), true);
            instruction_started = true;
        } else if (irq_pending()) {
            start_interrupt_micro_op(static_cast<uint8_t>(InterruptSource::Irq), true);
            instruction_started = true;
        } else {
            const uint8_t opcode = bus.peek8(regs_.pc);
            if (!start_micro_op(bus, opcode)) {
                return false;
            }
            instruction_started = true;
        }
    }

    signals = micro_op_signals();
    status = micro_op_status(instruction_started, false);
    return true;
}

CpuMicrocycleStatus Cpu::complete_microcycle(const BusSignals& signals) {
    if (!has_pending_micro_ops()) {
        return {};
    }

    const int32_t completed_step = micro_op_.step;
    switch (micro_op_.kind) {
    case MicroOpKind::Nop:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::MiscInherent:
        if (completed_step == 0) {
            switch (micro_op_.opcode) {
            case 0x19: {
                uint8_t adjust = 0;
                bool carry = (regs_.cc & CC_C) != 0;
                if ((regs_.a & 0x0F) > 9 || (regs_.cc & CC_H)) {
                    adjust |= 0x06;
                }
                if ((regs_.a > 0x99) || carry) {
                    adjust |= 0x60;
                    carry = true;
                }
                regs_.a = static_cast<uint8_t>(regs_.a + adjust);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (regs_.a & 0x80) regs_.cc |= CC_N;
                if (regs_.a == 0) regs_.cc |= CC_Z;
                if (carry) regs_.cc |= CC_C;
                break;
            }
            case 0x1D:
                regs_.b = (regs_.a & 0x80) ? 0xFF : 0x00;
                set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
                break;
            case 0x14: {
                const uint16_t w = reg_w();
                const uint32_t q = (w & 0x8000) ? (0xFFFF0000u | w) : w;
                set_reg_q(q);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (q == 0) regs_.cc |= CC_Z;
                if (q & 0x80000000u) regs_.cc |= CC_N;
                break;
            }
            case 0x3A:
                regs_.x = static_cast<uint16_t>(regs_.x + regs_.b);
                break;
            case 0x3D: {
                const uint16_t result = static_cast<uint16_t>(regs_.a) * static_cast<uint16_t>(regs_.b);
                regs_.a = hi(result);
                regs_.b = lo(result);
                regs_.cc &= static_cast<uint8_t>(~(CC_Z | CC_C | CC_V | CC_N));
                if (result == 0) regs_.cc |= CC_Z;
                if (result & 0x8000) regs_.cc |= CC_N;
                if (result & 0x80) regs_.cc |= CC_C;
                break;
            }
            default:
                break;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::CcImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            if (micro_op_.opcode == 0x1A) {
                regs_.cc |= signals.data;
            } else {
                regs_.cc &= signals.data;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::MdImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if (micro_op_.opcode == 0x3C) {
                const uint8_t value = static_cast<uint8_t>(regs_.md & signals.data);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
                if (value == 0) regs_.cc |= CC_Z;
                if (value & 0x80) regs_.cc |= CC_N;
            } else {
                regs_.md = signals.data;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::RegisterAlu:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            apply_register_alu(*this, regs_, micro_op_.opcode, signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::BitTransfer:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            uint8_t* regp = bit_transfer_register(regs_, micro_op_.direct_offset);
            if (!regp) break;
            if (micro_op_.opcode == 0x36) {
                const uint8_t src_bit = static_cast<uint8_t>((micro_op_.direct_offset >> 3) & 0x07);
                const uint8_t dst_bit = static_cast<uint8_t>(micro_op_.direct_offset & 0x07);
                const uint8_t mem_bit = static_cast<uint8_t>((signals.data >> src_bit) & 0x01);
                *regp = static_cast<uint8_t>((*regp & static_cast<uint8_t>(~(1u << dst_bit))) | (mem_bit << dst_bit));
            } else {
                micro_op_.data = apply_bit_transfer_memory_result(micro_op_.opcode, *regp, micro_op_.direct_offset, signals.data);
            }
        }
        break;
    case MicroOpKind::DivDImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (signals.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divd_result(regs_, signals.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::DivQImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (micro_op_.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divq_result(regs_, micro_op_.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::DivDDirect:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            if (signals.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divd_result(regs_, signals.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::DivQDirect:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divq_result(regs_, micro_op_.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 6) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::DivDExtended:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            if (signals.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divd_result(regs_, signals.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 6) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::DivQExtended:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divq_result(regs_, micro_op_.data);
            }
        } else if (micro_op_.branch_taken && completed_step == 6) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == 7) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::Tfm:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step >= 3 && completed_step < 3 + static_cast<int32_t>(2u * micro_op_.data32)) {
            const bool read_step = ((completed_step - 3) % 2) == 0;
            const uint8_t src_code = static_cast<uint8_t>(micro_op_.direct_offset >> 4);
            const uint8_t dst_code = static_cast<uint8_t>(micro_op_.direct_offset & 0x0F);
            if (read_step) {
                micro_op_.data = signals.data;
                if (uint16_t* source = tfm_register_pointer(regs_, src_code)) {
                    *source = static_cast<uint16_t>(*source + tfm_source_delta(micro_op_.opcode));
                }
            } else {
                if (uint16_t* destination = tfm_register_pointer(regs_, dst_code)) {
                    *destination = static_cast<uint16_t>(*destination + tfm_destination_delta(micro_op_.opcode));
                }
                set_reg_w(static_cast<uint16_t>(reg_w() - 1));
            }
        }
        break;
    case MicroOpKind::RegisterTransfer:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            const uint8_t src = static_cast<uint8_t>(signals.data >> 4);
            const uint8_t dst = static_cast<uint8_t>(signals.data & 0x0F);
            if (micro_op_.opcode == 0x1F) {
                const bool dest16 = register_code_is_16bit(dst);
                const uint16_t value = read_reg_for_dest(regs_, src, dest16);
                write_reg_sized(*this, dst, value, dest16);
            } else {
                const bool wide = register_code_is_16bit(src) || register_code_is_16bit(dst);
                const uint16_t left = read_reg_for_dest(regs_, src, wide);
                const uint16_t right = read_reg_for_dest(regs_, dst, wide);
                write_reg_sized(*this, src, right, wide);
                write_reg_sized(*this, dst, left, wide);
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::PrefixedRegisterUnary:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            if (micro_op_.prefix == 0x10 && micro_op_.opcode < 0x50) {
                const uint16_t value = reg_d_value(regs_);
                switch (micro_op_.opcode) {
                case 0x40: set_reg_d_value(regs_, neg16_unary(regs_, value)); break;
                case 0x43: set_reg_d_value(regs_, com16_unary(regs_, value)); break;
                case 0x44: set_reg_d_value(regs_, lsr16_unary(regs_, value)); break;
                case 0x46: set_reg_d_value(regs_, ror16_unary(regs_, value)); break;
                case 0x47: set_reg_d_value(regs_, asr16_unary(regs_, value)); break;
                case 0x48: set_reg_d_value(regs_, lsl16_unary(regs_, value)); break;
                case 0x49: set_reg_d_value(regs_, rol16_unary(regs_, value)); break;
                case 0x4A: set_reg_d_value(regs_, dec16_unary(regs_, value)); break;
                case 0x4C: set_reg_d_value(regs_, inc16_unary(regs_, value)); break;
                case 0x4D: set_tst16_flags(regs_, value); break;
                case 0x4F:
                    set_reg_d_value(regs_, 0);
                    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                    regs_.cc |= CC_Z;
                    break;
                default:
                    break;
                }
            } else if (micro_op_.prefix == 0x10) {
                const uint16_t value = reg_w();
                switch (micro_op_.opcode) {
                case 0x53: set_reg_w(com16_unary(regs_, value)); break;
                case 0x54: set_reg_w(lsr16_unary(regs_, value)); break;
                case 0x56: set_reg_w(ror16_unary(regs_, value)); break;
                case 0x58: set_reg_w(lsl16_unary(regs_, value)); break;
                case 0x59: set_reg_w(rol16_unary(regs_, value)); break;
                case 0x5A: set_reg_w(dec16_unary(regs_, value)); break;
                case 0x5C: set_reg_w(inc16_unary(regs_, value)); break;
                case 0x5D: set_tst16_flags(regs_, value); break;
                case 0x5F:
                    set_reg_w(0);
                    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                    regs_.cc |= CC_Z;
                    break;
                default:
                    break;
                }
            } else {
                uint8_t& target = micro_op_.opcode < 0x50 ? regs_.e : regs_.f;
                switch (micro_op_.opcode & 0x0F) {
                case 0x03:
                case 0x0A:
                case 0x0C:
                    target = apply_memory_unary(regs_, micro_op_.opcode, target);
                    break;
                case 0x0D:
                    apply_tst8(regs_, target);
                    break;
                case 0x0F:
                    target = 0;
                    regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                    regs_.cc |= CC_Z;
                    break;
                default:
                    break;
                }
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::EFAluImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            apply_ef_alu8(regs_, micro_op_.opcode, signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::LoadImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (micro_op_.width == MicroOpWidth::Byte && completed_step == 1) {
            if (micro_op_.target == MicroOpTarget::A) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else if (micro_op_.target == MicroOpTarget::B) {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (micro_op_.width == MicroOpWidth::Word) {
            if (completed_step == 1) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (completed_step == 2) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                if (micro_op_.target == MicroOpTarget::D) {
                    regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
                    regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
                    set_flags_nz16(micro_op_.data);
                }
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (micro_op_.width == MicroOpWidth::Long) {
            micro_op_.data32 = static_cast<uint32_t>((micro_op_.data32 << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (completed_step == static_cast<int32_t>(micro_op_.width) && micro_op_.target == MicroOpTarget::Q) {
                set_reg_q(micro_op_.data32);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (micro_op_.data32 == 0) regs_.cc |= CC_Z;
                if (micro_op_.data32 & 0x80000000u) regs_.cc |= CC_N;
            }
        }
        break;
    case MicroOpKind::Alu8Immediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Alu16Immediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::CmpdImmediate:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmpd(regs_, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::DAluImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_hd6309_d_alu(regs_, micro_op_.opcode, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Cmp16Immediate: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    }
    case MicroOpKind::WordLoadImmediate: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 2 : 3;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
            set_flags_nz16(micro_op_.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    }
    case MicroOpKind::WLoadImmediate:
    case MicroOpKind::WAluImmediate:
    case MicroOpKind::WCmpImmediate:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.kind == MicroOpKind::WLoadImmediate) {
                set_reg_w(micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::WAluImmediate) {
                const uint16_t result =
                    micro_op_.opcode == 0x8B ? add16(regs_, reg_w(), micro_op_.data) : sub16(regs_, reg_w(), micro_op_.data);
                set_reg_w(result);
            } else {
                sub16(regs_, reg_w(), micro_op_.data);
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::Alu8Direct:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
        }
        break;
    case MicroOpKind::Alu16Direct:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
        }
        break;
    case MicroOpKind::CmpdDirect:
    case MicroOpKind::DAluDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.kind == MicroOpKind::DAluDirect) {
                apply_hd6309_d_alu(regs_, micro_op_.opcode, micro_op_.data);
            } else {
                apply_cmpd(regs_, micro_op_.data);
            }
        }
        break;
    case MicroOpKind::Cmp16Direct: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
        }
        break;
    }
    case MicroOpKind::WLoadDirect:
    case MicroOpKind::WAluDirect:
    case MicroOpKind::WCmpDirect:
    case MicroOpKind::WStoreDirect: {
        const uint8_t operand_step = 2;
        const uint8_t high_step = 3;
        const uint8_t low_step = 4;
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step && micro_op_.kind != MicroOpKind::WStoreDirect) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_step) {
            if (micro_op_.kind == MicroOpKind::WStoreDirect) {
                set_flags_nz16(reg_w());
            } else {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                if (micro_op_.kind == MicroOpKind::WLoadDirect) {
                    set_reg_w(micro_op_.data);
                    set_flags_nz16(micro_op_.data);
                } else if (micro_op_.kind == MicroOpKind::WAluDirect) {
                    const uint16_t result =
                        micro_op_.opcode == 0x9B ? add16(regs_, reg_w(), micro_op_.data) : sub16(regs_, reg_w(), micro_op_.data);
                    set_reg_w(result);
                } else {
                    sub16(regs_, reg_w(), micro_op_.data);
                }
            }
        }
        break;
    }
    case MicroOpKind::EFAluDirect:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            apply_ef_alu8(regs_, micro_op_.opcode, signals.data);
        }
        break;
    case MicroOpKind::QLoadDirect:
    case MicroOpKind::QStoreDirect: {
        const uint8_t operand_step = 2;
        const uint8_t data_step = 3;
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step >= data_step && completed_step < data_step + 4) {
            if (micro_op_.kind == MicroOpKind::QLoadDirect) {
                micro_op_.data32 = static_cast<uint32_t>((micro_op_.data32 << 8) | signals.data);
            }
            if (completed_step == data_step + 3) {
                const uint32_t value = micro_op_.kind == MicroOpKind::QLoadDirect ? micro_op_.data32 : reg_q();
                if (micro_op_.kind == MicroOpKind::QLoadDirect) set_reg_q(value);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (value == 0) regs_.cc |= CC_Z;
                if (value & 0x80000000u) regs_.cc |= CC_N;
            }
        }
        break;
    }
    case MicroOpKind::WordLoadDirect:
    case MicroOpKind::WordStoreDirect: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t operand_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t high_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t low_step = micro_op_.prefix == 0x00 ? 3 : 4;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == operand_step) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_step && micro_op_.kind == MicroOpKind::WordLoadDirect) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_step) {
            if (micro_op_.kind == MicroOpKind::WordLoadDirect) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else {
                set_flags_nz16(word_register_value(regs_, micro_op_.prefix, micro_op_.opcode));
            }
        }
        break;
    }
    case MicroOpKind::LoadDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if (micro_op_.width == MicroOpWidth::Word) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.target == MicroOpTarget::A) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
        } else if (completed_step == 3 && micro_op_.width == MicroOpWidth::Word) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
            regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
            set_flags_nz16(micro_op_.data);
        }
        break;
    case MicroOpKind::LoadExtended:
    case MicroOpKind::Alu8Extended:
    case MicroOpKind::Alu16Extended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            if (micro_op_.kind == MicroOpKind::Alu8Extended) {
                apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
            } else if (micro_op_.kind == MicroOpKind::Alu16Extended) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.width == MicroOpWidth::Word) {
                micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            } else if (micro_op_.target == MicroOpTarget::A) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
        } else if (completed_step == 4) {
            if (micro_op_.kind == MicroOpKind::Alu16Extended) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::LoadExtended && micro_op_.width == MicroOpWidth::Word) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                regs_.a = static_cast<uint8_t>((micro_op_.data >> 8) & 0xFF);
                regs_.b = static_cast<uint8_t>(micro_op_.data & 0xFF);
                set_flags_nz16(micro_op_.data);
            }
        }
        break;
    case MicroOpKind::CmpdExtended:
    case MicroOpKind::DAluExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 5) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.kind == MicroOpKind::DAluExtended) {
                apply_hd6309_d_alu(regs_, micro_op_.opcode, micro_op_.data);
            } else {
                apply_cmpd(regs_, micro_op_.data);
            }
        }
        break;
    case MicroOpKind::Cmp16Extended: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_data_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_data_step) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
        }
        break;
    }
    case MicroOpKind::WLoadExtended:
    case MicroOpKind::WAluExtended:
    case MicroOpKind::WCmpExtended:
    case MicroOpKind::WStoreExtended: {
        const uint8_t high_address_step = 2;
        const uint8_t low_address_step = 3;
        const uint8_t high_data_step = 4;
        const uint8_t low_data_step = 5;
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_data_step && micro_op_.kind != MicroOpKind::WStoreExtended) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_data_step) {
            if (micro_op_.kind == MicroOpKind::WStoreExtended) {
                set_flags_nz16(reg_w());
            } else {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                if (micro_op_.kind == MicroOpKind::WLoadExtended) {
                    set_reg_w(micro_op_.data);
                    set_flags_nz16(micro_op_.data);
                } else if (micro_op_.kind == MicroOpKind::WAluExtended) {
                    const uint16_t result =
                        micro_op_.opcode == 0xBB ? add16(regs_, reg_w(), micro_op_.data) : sub16(regs_, reg_w(), micro_op_.data);
                    set_reg_w(result);
                } else {
                    sub16(regs_, reg_w(), micro_op_.data);
                }
            }
        }
        break;
    }
    case MicroOpKind::EFAluExtended:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            apply_ef_alu8(regs_, micro_op_.opcode, signals.data);
        }
        break;
    case MicroOpKind::QLoadExtended:
    case MicroOpKind::QStoreExtended: {
        const uint8_t high_address_step = 2;
        const uint8_t low_address_step = 3;
        const uint8_t data_step = 4;
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step >= data_step && completed_step < data_step + 4) {
            if (micro_op_.kind == MicroOpKind::QLoadExtended) {
                micro_op_.data32 = static_cast<uint32_t>((micro_op_.data32 << 8) | signals.data);
            }
            if (completed_step == data_step + 3) {
                const uint32_t value = micro_op_.kind == MicroOpKind::QLoadExtended ? micro_op_.data32 : reg_q();
                if (micro_op_.kind == MicroOpKind::QLoadExtended) set_reg_q(value);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (value == 0) regs_.cc |= CC_Z;
                if (value & 0x80000000u) regs_.cc |= CC_N;
            }
        }
        break;
    }
    case MicroOpKind::WordLoadExtended:
    case MicroOpKind::WordStoreExtended: {
        const uint8_t prefix_step = micro_op_.prefix == 0x00 ? 0xFF : 1;
        const uint8_t high_address_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t low_address_step = micro_op_.prefix == 0x00 ? 2 : 3;
        const uint8_t high_data_step = micro_op_.prefix == 0x00 ? 3 : 4;
        const uint8_t low_data_step = micro_op_.prefix == 0x00 ? 4 : 5;
        if (completed_step == 0 || completed_step == prefix_step) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == low_address_step) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == high_data_step && micro_op_.kind == MicroOpKind::WordLoadExtended) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == low_data_step) {
            if (micro_op_.kind == MicroOpKind::WordLoadExtended) {
                micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
                set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else {
                set_flags_nz16(word_register_value(regs_, micro_op_.prefix, micro_op_.opcode));
            }
        }
        break;
    }
    case MicroOpKind::ImmediateMemoryDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data32 = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            const uint8_t mask = static_cast<uint8_t>(micro_op_.data32 & 0xFF);
            const uint8_t result = apply_immediate_memory_value(micro_op_.opcode, signals.data, mask);
            micro_op_.data = result;
            set_flags_nz8(result);
            regs_.cc &= static_cast<uint8_t>(~CC_V);
        }
        break;
    case MicroOpKind::ImmediateMemoryExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data32 = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 4) {
            const uint8_t mask = static_cast<uint8_t>(micro_op_.data32 & 0xFF);
            const uint8_t result = apply_immediate_memory_value(micro_op_.opcode, signals.data, mask);
            micro_op_.data = result;
            set_flags_nz8(result);
            regs_.cc &= static_cast<uint8_t>(~CC_V);
        }
        break;
    case MicroOpKind::IndexedImmediateMemory: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(3 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data32 = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == 3) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == 4) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step) {
            const uint8_t mask = static_cast<uint8_t>(micro_op_.data32 & 0xFF);
            const uint8_t result = apply_immediate_memory_value(micro_op_.opcode, signals.data, mask);
            micro_op_.data = result;
            set_flags_nz8(result);
            regs_.cc &= static_cast<uint8_t>(~CC_V);
        }
        break;
    }
    case MicroOpKind::MemoryUnaryDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if ((micro_op_.opcode & 0x0F) == 0x0F) {
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                regs_.cc |= CC_Z;
            } else if ((micro_op_.opcode & 0x0F) == 0x0D) {
                apply_tst8(regs_, signals.data);
            } else {
                micro_op_.data = apply_memory_unary(regs_, micro_op_.opcode, signals.data);
            }
        }
        break;
    case MicroOpKind::MemoryUnaryExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            if ((micro_op_.opcode & 0x0F) == 0x0F) {
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                regs_.cc |= CC_Z;
            } else if ((micro_op_.opcode & 0x0F) == 0x0D) {
                apply_tst8(regs_, signals.data);
            } else {
                micro_op_.data = apply_memory_unary(regs_, micro_op_.opcode, signals.data);
            }
        }
        break;
    case MicroOpKind::IndexedLoad8:
    case MicroOpKind::IndexedLoad16:
    case MicroOpKind::IndexedStore8:
    case MicroOpKind::IndexedStore16:
    case MicroOpKind::IndexedAlu8:
    case MicroOpKind::IndexedMemoryUnary: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint8_t low_opcode = static_cast<uint8_t>(micro_op_.opcode & 0x0F);
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == 2) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedMemoryUnary) {
            if (low_opcode == 0x0F) {
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_C));
                regs_.cc |= CC_Z;
            } else if (low_opcode == 0x0D) {
                apply_tst8(regs_, signals.data);
            } else {
                micro_op_.data = apply_memory_unary(regs_, micro_op_.opcode, signals.data);
            }
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedAlu8) {
            apply_alu8_immediate(regs_, micro_op_.opcode, signals.data);
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedLoad8) {
            if (micro_op_.opcode == 0xA6) {
                regs_.a = signals.data;
                set_flags_nz8(regs_.a);
            } else {
                regs_.b = signals.data;
                set_flags_nz8(regs_.b);
            }
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedLoad16) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == operand_step + 1 && micro_op_.kind == MicroOpKind::IndexedLoad16) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.a = hi(micro_op_.data);
            regs_.b = lo(micro_op_.data);
            set_flags_nz16(micro_op_.data);
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedStore8) {
            const uint8_t value = micro_op_.opcode == 0xE7 ? regs_.b : regs_.a;
            set_flags_nz8(value);
        } else if (completed_step == operand_step + 1 && micro_op_.kind == MicroOpKind::IndexedStore16) {
            set_flags_nz16(reg_d_value(regs_));
        }
        break;
    }
    case MicroOpKind::IndexedAlu16:
    case MicroOpKind::IndexedCmpd:
    case MicroOpKind::IndexedDAlu:
    case MicroOpKind::IndexedCmp16:
    case MicroOpKind::IndexedWordLoad:
    case MicroOpKind::IndexedWLoad:
    case MicroOpKind::IndexedWAlu:
    case MicroOpKind::IndexedWCmp: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0 || (micro_op_.prefix != 0x00 && completed_step == 1)) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == operand_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.kind == MicroOpKind::IndexedAlu16) {
                apply_alu16(regs_, micro_op_.opcode, micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::IndexedCmpd) {
                apply_cmpd(regs_, micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::IndexedDAlu) {
                apply_hd6309_d_alu(regs_, micro_op_.opcode, micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::IndexedWordLoad) {
                set_word_register(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::IndexedWLoad) {
                set_reg_w(micro_op_.data);
                set_flags_nz16(micro_op_.data);
            } else if (micro_op_.kind == MicroOpKind::IndexedWAlu) {
                const uint16_t result =
                    micro_op_.opcode == 0xAB ? add16(regs_, reg_w(), micro_op_.data) : sub16(regs_, reg_w(), micro_op_.data);
                set_reg_w(result);
            } else if (micro_op_.kind == MicroOpKind::IndexedWCmp) {
                sub16(regs_, reg_w(), micro_op_.data);
            } else {
                apply_cmp16(regs_, micro_op_.prefix, micro_op_.opcode, micro_op_.data);
            }
        }
        break;
    }
    case MicroOpKind::IndexedEFAlu: {
        const uint8_t post_step = 2;
        const uint8_t extension_start_step = 3;
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step) {
            apply_ef_alu8(regs_, micro_op_.opcode, signals.data);
        }
        break;
    }
    case MicroOpKind::IndexedDivD:
    case MicroOpKind::IndexedDivQ: {
        const uint8_t post_step = 2;
        const uint8_t extension_start_step = 3;
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint8_t vector_step = static_cast<uint8_t>(operand_step + (micro_op_.kind == MicroOpKind::IndexedDivQ ? 2 : 1));
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedDivD) {
            if (signals.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divd_result(regs_, signals.data);
            }
        } else if (completed_step == operand_step && micro_op_.kind == MicroOpKind::IndexedDivQ) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == operand_step + 1 && micro_op_.kind == MicroOpKind::IndexedDivQ) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            if (micro_op_.data == 0) {
                regs_.md |= 0x80;
                micro_op_.branch_taken = true;
            } else {
                apply_divq_result(regs_, micro_op_.data);
            }
        } else if (micro_op_.branch_taken && completed_step == vector_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.branch_taken && completed_step == vector_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    }
    case MicroOpKind::IndexedQLoad: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0 || (micro_op_.prefix != 0x00 && completed_step == 1)) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step >= operand_step && completed_step < operand_step + 4) {
            micro_op_.data32 = static_cast<uint32_t>((micro_op_.data32 << 8) | signals.data);
            if (completed_step == operand_step + 3) {
                set_reg_q(micro_op_.data32);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (micro_op_.data32 == 0) regs_.cc |= CC_Z;
                if (micro_op_.data32 & 0x80000000u) regs_.cc |= CC_N;
            }
        }
        break;
    }
    case MicroOpKind::IndexedWordStore: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0 || (micro_op_.prefix != 0x00 && completed_step == 1)) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step + 1) {
            set_flags_nz16(word_register_value(regs_, micro_op_.prefix, micro_op_.opcode));
        }
        break;
    }
    case MicroOpKind::IndexedWStore:
    case MicroOpKind::IndexedQStore: {
        const uint8_t post_step = micro_op_.prefix == 0x00 ? 1 : 2;
        const uint8_t extension_start_step = static_cast<uint8_t>(post_step + 1);
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(extension_start_step + extension_bytes);
        const uint8_t operand_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        const uint8_t operand_bytes = micro_op_.kind == MicroOpKind::IndexedQStore ? 4 : 2;
        if (completed_step == 0 || (micro_op_.prefix != 0x00 && completed_step == 1)) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == post_step) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == extension_start_step) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == extension_start_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == extension_start_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == operand_step + operand_bytes - 1) {
            if (micro_op_.kind == MicroOpKind::IndexedQStore) {
                const uint32_t value = reg_q();
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z));
                if (value == 0) regs_.cc |= CC_Z;
                if (value & 0x80000000u) regs_.cc |= CC_N;
            } else {
                set_flags_nz16(reg_w());
            }
        }
        break;
    }
    case MicroOpKind::StoreDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2 && micro_op_.width == MicroOpWidth::Byte) {
            const uint8_t value = micro_op_.target == MicroOpTarget::A ? regs_.a : regs_.b;
            set_flags_nz8(value);
        } else if (completed_step == 3 && micro_op_.width == MicroOpWidth::Word) {
            set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
        }
        break;
    case MicroOpKind::StoreExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3 && micro_op_.width == MicroOpWidth::Byte) {
            const uint8_t value = micro_op_.target == MicroOpTarget::A ? regs_.a : regs_.b;
            set_flags_nz8(value);
        } else if (completed_step == 4 && micro_op_.width == MicroOpWidth::Word) {
            set_flags_nz16(static_cast<uint16_t>((static_cast<uint16_t>(regs_.a) << 8) | regs_.b));
        }
        break;
    case MicroOpKind::Sync:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            sync_wait_ = true;
        }
        break;
    case MicroOpKind::InterruptVector: {
        const auto source = static_cast<InterruptSource>(micro_op_.opcode);
        if (completed_step == 0) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            apply_interrupt_masks(regs_, source);
            regs_.pc = micro_op_.data;
            if (source == InterruptSource::Nmi) {
                nmi_latched_ = false;
            }
            sync_wait_ = false;
            cwai_wait_ = false;
        }
        break;
    }
    case MicroOpKind::IrqEntry:
    case MicroOpKind::Swi: {
        const bool external_interrupt = micro_op_.kind == MicroOpKind::IrqEntry;
        const auto source = static_cast<InterruptSource>(micro_op_.opcode);
        const int32_t stack_start = external_interrupt ? 0 : (micro_op_.prefix == 0x00 ? 1 : 2);
        const int32_t stack_bytes = external_interrupt
            ? interrupt_stack_byte_count(regs_, mode_, source)
            : (native_hd6309_frame(regs_, mode_) ? 14 : 12);
        const int32_t vector_step = stack_start + stack_bytes;
        if (micro_op_.kind == MicroOpKind::Swi &&
            (completed_step == 0 || (micro_op_.prefix != 0x00 && completed_step == 1))) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if ((micro_op_.prefix == 0x00 && completed_step == 0) ||
                (micro_op_.prefix != 0x00 && completed_step == 1)) {
                regs_.cc = static_cast<uint8_t>(regs_.cc | CC_E);
            }
        } else if (completed_step >= stack_start && completed_step < vector_step) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            if (completed_step == vector_step - 1) {
                if (micro_op_.kind == MicroOpKind::Swi) {
                    regs_.cc = static_cast<uint8_t>(regs_.cc | CC_I);
                    if (micro_op_.prefix == 0x00) {
                        regs_.cc = static_cast<uint8_t>(regs_.cc | CC_F);
                    }
                } else {
                    apply_interrupt_masks(regs_, source);
                    if (source == InterruptSource::Nmi) {
                        nmi_latched_ = false;
                    }
                    sync_wait_ = false;
                    cwai_wait_ = false;
                }
            }
        } else if (completed_step == vector_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == vector_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = micro_op_.data;
        }
        break;
    }
    case MicroOpKind::Cwai: {
        constexpr int32_t stack_start = 2;
        const int32_t stack_end = stack_start + (native_hd6309_frame(regs_, mode_) ? 14 : 12);
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.cc = static_cast<uint8_t>((regs_.cc & signals.data) | CC_E);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step >= stack_start && completed_step < stack_end) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            if (completed_step == stack_end - 1) {
                cwai_wait_ = true;
            }
        }
        break;
    }
    case MicroOpKind::Rti: {
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else {
            const bool native_full_frame = micro_op_.total_cycles == 17;
            const uint8_t mask = micro_op_.total_cycles == 6 ? 0x81 : 0xFF;
            const uint8_t byte_index = static_cast<uint8_t>(completed_step - 1);
            const uint8_t byte_count = native_full_frame ? 14 : stack_mask_byte_count(mask);
            if (byte_index < byte_count) {
                if (native_full_frame) {
                    apply_native_interrupt_pull_byte(regs_, byte_index, signals.data, micro_op_.data);
                } else {
                    apply_stack_pull_byte(regs_, mask, byte_index, false, signals.data, micro_op_.data);
                }
                regs_.s = static_cast<uint16_t>(regs_.s + 1);
            }
        }
        break;
    }
    case MicroOpKind::Branch:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (micro_op_.branch_taken) {
                regs_.pc = static_cast<uint16_t>(regs_.pc + static_cast<int8_t>(signals.data));
            }
        }
        break;
    case MicroOpKind::Lbra:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            regs_.pc = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
        }
        break;
    case MicroOpKind::LongBranch:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 3) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (micro_op_.branch_taken) {
                regs_.pc = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
            }
        }
        break;
    case MicroOpKind::Bsr:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
            micro_op_.effective_address = static_cast<uint16_t>(regs_.pc + static_cast<int8_t>(signals.data));
        } else if (completed_step == 2) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::Lbsr:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            const uint16_t offset = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
            micro_op_.effective_address = static_cast<uint16_t>(regs_.pc + sign_extend16(offset));
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 4) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::Rts:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.s = static_cast<uint16_t>(regs_.s + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.s = static_cast<uint16_t>(regs_.s + 1);
            regs_.pc = micro_op_.data;
        }
        break;
    case MicroOpKind::StackPush:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else {
            const uint8_t byte_index = static_cast<uint8_t>(completed_step - 2);
            if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
                if (micro_op_.opcode == 0x36) {
                    regs_.u = static_cast<uint16_t>(regs_.u - 1);
                } else {
                    regs_.s = static_cast<uint16_t>(regs_.s - 1);
                }
            }
        }
        break;
    case MicroOpKind::StackPull:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else {
            const uint8_t byte_index = static_cast<uint8_t>(completed_step - 2);
            if (byte_index < stack_mask_byte_count(micro_op_.direct_offset)) {
                const bool use_u_stack = micro_op_.opcode == 0x37;
                apply_stack_pull_byte(regs_, micro_op_.direct_offset, byte_index, use_u_stack, signals.data, micro_op_.data);
                if (use_u_stack) {
                    regs_.u = static_cast<uint16_t>(regs_.u + 1);
                } else {
                    regs_.s = static_cast<uint16_t>(regs_.s + 1);
                }
            }
        }
        break;
    case MicroOpKind::WStackPush:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            if (micro_op_.opcode == 0x3A) {
                regs_.u = static_cast<uint16_t>(regs_.u - 2);
            } else {
                regs_.s = static_cast<uint16_t>(regs_.s - 1);
            }
        } else if (completed_step == 3) {
            if (micro_op_.opcode == 0x38) {
                regs_.s = static_cast<uint16_t>(regs_.s - 1);
            }
        }
        break;
    case MicroOpKind::WStackPull:
        if (completed_step == 0 || completed_step == 1) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            set_reg_w(micro_op_.data);
            if (micro_op_.opcode == 0x3B) {
                regs_.u = static_cast<uint16_t>(regs_.u + 2);
            } else {
                regs_.s = static_cast<uint16_t>(regs_.s + 2);
            }
        }
        break;
    case MicroOpKind::JsrDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
        } else if (completed_step == 2) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JsrExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.data = regs_.pc;
        } else if (completed_step == 3) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == 4) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JsrIndexed: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        const uint8_t stack_low_step = static_cast<uint8_t>(pointer_step + (micro_op_.indexed_indirect ? 2 : 0));
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
            }
        } else if (extension_bytes == 1 && completed_step == 2) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (extension_bytes == 2 && completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
        } else if (completed_step == stack_low_step) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
        } else if (completed_step == stack_low_step + 1) {
            regs_.s = static_cast<uint16_t>(regs_.s - 1);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    }
    case MicroOpKind::JmpDirect:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>((static_cast<uint16_t>(regs_.dp) << 8) | signals.data);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JmpExtended:
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.effective_address = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 2) {
            micro_op_.effective_address = static_cast<uint16_t>(micro_op_.effective_address | signals.data);
            regs_.pc = micro_op_.effective_address;
        }
        break;
    case MicroOpKind::JmpIndexed: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
                if (!micro_op_.indexed_indirect) {
                    regs_.pc = micro_op_.effective_address;
                }
            }
        } else if (extension_bytes == 1 && completed_step == 2) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
            if (!micro_op_.indexed_indirect) {
                regs_.pc = micro_op_.effective_address;
            }
        } else if (extension_bytes == 2 && completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
            if (!micro_op_.indexed_indirect) {
                regs_.pc = micro_op_.effective_address;
            }
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
            regs_.pc = micro_op_.effective_address;
        }
        break;
    }
    case MicroOpKind::IndexedLea: {
        const uint8_t extension_bytes = indexed_extension_byte_count(micro_op_.direct_offset);
        const uint8_t pointer_step = static_cast<uint8_t>(2 + extension_bytes);
        if (completed_step == 0) {
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (completed_step == 1) {
            micro_op_.direct_offset = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            if (indexed_extension_byte_count(micro_op_.direct_offset) == 0) {
                micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, 0);
                if (!micro_op_.indexed_indirect) {
                    set_lea_register_value(regs_, micro_op_.opcode, micro_op_.effective_address);
                    if (lea_sets_flags(micro_op_.opcode)) set_flags_nz16(micro_op_.effective_address);
                }
            }
        } else if (extension_bytes == 1 && completed_step == 2) {
            micro_op_.data = signals.data;
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
            if (!micro_op_.indexed_indirect) {
                set_lea_register_value(regs_, micro_op_.opcode, micro_op_.effective_address);
                if (lea_sets_flags(micro_op_.opcode)) set_flags_nz16(micro_op_.effective_address);
            }
        } else if (extension_bytes == 2 && completed_step == 2) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        } else if (extension_bytes == 2 && completed_step == 3) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
            micro_op_.effective_address = indexed_effective_address_and_update(regs_, micro_op_.direct_offset, micro_op_.data);
            if (!micro_op_.indexed_indirect) {
                set_lea_register_value(regs_, micro_op_.opcode, micro_op_.effective_address);
                if (lea_sets_flags(micro_op_.opcode)) set_flags_nz16(micro_op_.effective_address);
            }
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step) {
            micro_op_.data = static_cast<uint16_t>(static_cast<uint16_t>(signals.data) << 8);
        } else if (micro_op_.indexed_indirect && completed_step == pointer_step + 1) {
            micro_op_.data = static_cast<uint16_t>(micro_op_.data | signals.data);
            micro_op_.effective_address = micro_op_.data;
            set_lea_register_value(regs_, micro_op_.opcode, micro_op_.effective_address);
            if (lea_sets_flags(micro_op_.opcode)) set_flags_nz16(micro_op_.effective_address);
        }
        break;
    }
    case MicroOpKind::RegisterUnary:
        if (completed_step == 0) {
            uint8_t& target = micro_op_.target == MicroOpTarget::A ? regs_.a : regs_.b;
            switch (micro_op_.opcode & 0x0F) {
            case 0x0: {
                const uint8_t operand = target;
                target = static_cast<uint8_t>(0u - operand);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target == 0) regs_.cc |= CC_Z;
                if (target & 0x80) regs_.cc |= CC_N;
                if (operand == 0x80) regs_.cc |= CC_V;
                if (operand != 0) regs_.cc |= CC_C;
                break;
            }
            case 0x3:
                target = static_cast<uint8_t>(~target);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
                regs_.cc |= CC_C;
                if (target == 0) regs_.cc |= CC_Z;
                if (target & 0x80) regs_.cc |= CC_N;
                break;
            case 0x4: {
                const uint8_t carry = static_cast<uint8_t>(target & 0x01);
                target = static_cast<uint8_t>(target >> 1);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (carry != 0) regs_.cc |= CC_C;
                if (target == 0) regs_.cc |= CC_Z;
                break;
            }
            case 0x6: {
                const uint8_t carry_in = (regs_.cc & CC_C) ? 0x80 : 0x00;
                const uint8_t carry_out = static_cast<uint8_t>(target & 0x01);
                target = static_cast<uint8_t>((target >> 1) | carry_in);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target & 0x80) regs_.cc |= CC_N;
                if (target == 0) regs_.cc |= CC_Z;
                if (carry_out != 0) regs_.cc |= CC_C;
                if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs_.cc |= CC_V;
                break;
            }
            case 0x7: {
                const uint8_t carry_out = static_cast<uint8_t>(target & 0x01);
                target = static_cast<uint8_t>((target >> 1) | (target & 0x80));
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target & 0x80) regs_.cc |= CC_N;
                if (target == 0) regs_.cc |= CC_Z;
                if (carry_out != 0) regs_.cc |= CC_C;
                break;
            }
            case 0x8: {
                const uint8_t carry_out = static_cast<uint8_t>((target >> 7) & 0x01);
                target = static_cast<uint8_t>(target << 1);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target & 0x80) regs_.cc |= CC_N;
                if (target == 0) regs_.cc |= CC_Z;
                if (carry_out != 0) regs_.cc |= CC_C;
                if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs_.cc |= CC_V;
                break;
            }
            case 0x9: {
                const uint8_t carry_in = (regs_.cc & CC_C) ? 1 : 0;
                const uint8_t carry_out = static_cast<uint8_t>((target >> 7) & 0x01);
                target = static_cast<uint8_t>((target << 1) | carry_in);
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target & 0x80) regs_.cc |= CC_N;
                if (target == 0) regs_.cc |= CC_Z;
                if (carry_out != 0) regs_.cc |= CC_C;
                if (((target ^ (carry_out ? 0x80 : 0x00)) & 0x80) != 0) regs_.cc |= CC_V;
                break;
            }
            case 0xA:
            case 0xC: {
                const bool increment = (micro_op_.opcode & 0x0F) == 0xC;
                const uint8_t result = static_cast<uint8_t>(target + (increment ? 1 : -1));
                target = result;
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V));
                if (result == 0) regs_.cc |= CC_Z;
                if (result & 0x80) regs_.cc |= CC_N;
                if ((increment && result == 0x80) || (!increment && result == 0x7F)) regs_.cc |= CC_V;
                break;
            }
            case 0xD:
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_Z | CC_V | CC_C));
                if (target == 0) regs_.cc |= CC_Z;
                if (target & 0x80) regs_.cc |= CC_N;
                break;
            case 0xF:
                target = 0;
                regs_.cc &= static_cast<uint8_t>(~(CC_N | CC_V | CC_Z));
                regs_.cc |= CC_Z;
                break;
            default:
                break;
            }
            regs_.pc = static_cast<uint16_t>(regs_.pc + 1);
        }
        break;
    case MicroOpKind::None:
        break;
    }

    micro_op_.step += 1;
    const bool complete = micro_op_.step >= micro_op_.total_cycles;
    const CpuMicrocycleStatus status = micro_op_status(false, complete);
    if (complete) {
        cycles_executed_ += micro_op_.total_cycles;
        micro_op_ = {};
    }
    return status;
}

} // namespace microlind
